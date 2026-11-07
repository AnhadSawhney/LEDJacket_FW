# Phone Companion App & USB Frame Protocol Specification

**Component**: Android Companion App (Host Frame Generator & Telemetry Receiver)  
**Physical Link**: USB 2.0 High-Speed (`480 Mbps`) to Central Controller `FTDI FT2232H` Channel A (`60 MHz` 8-bit Synchronous FIFO, `40 MB/s` sustained)  
**Alternative Input**: Gigabit Ethernet (`1000BASE-T` via `Colorlight i9` Broadcom `B50612D` PHY) for direct laptop/console Art-Net / sACN / Raw UDP streaming  

---

## 1. Architectural Scope & Input Assumptions

For the purposes of the firmware and gateware pipeline, **frame generation is treated as an upstream black box**:
- Whatever shader, video decoder, or stage effect is running on the phone has already evaluated its 2D/3D spatial mapping onto the jacket's triangular mesh.
- At the end of each render tick ($60\text{ Hz}$ nominal, up to $120\text{ Hz}$ in `RGB565` / `LUT8` modes), the phone holds an array of desired `(R, G, B)` colors for every physical LED on the jacket:
  - **Single-Triangle Logical Unit**: $66\text{ LEDs}$ per base-11 triangle ($\sum_{k=1}^{11} k = 66$).
  - **Dual-Triangle Rigid-Flex Node (`TriangleNode` v2.0)**: $2 \times 66 = \mathbf{132\text{ LEDs}}$ per MCU across 8 parallel LED strings (`PA0..PA7`).
  - **System Scale**: Up to **70 RS-485 daisy chains** (`CHAIN_00` .. `CHAIN_69`), supporting up to **850 Dual-Triangle Nodes** ($1,700$ triangles = $112,200\text{ LEDs}$), or proportional subsets depending on garment population.

This document specifies:
1. **Where along the pipeline** (`Phone` vs. `FPGA` vs. `TriangleNode MCU`) the logical triangle LED coordinates get reshuffled into the physical 8-string serpentine wiring order of each node.
2. **How display frames are formatted and packed** for transmission over the USB port into the FPGA.
3. **Whether frames should be compressed**, analyzing both the Phone-to-FPGA USB link and the optional Stage-Laptop-to-Phone Wi-Fi 6 link.
4. **How upstream telemetry** (5× IMU quaternions, USB-PD power contract, GaN buck current, and node temperatures) returns from the FPGA to the phone over USB.

---

## 2. Where Does LED Order Reshuffling Happen?

Within each `TriangleNode` v2.0 board, the 132 addressable 1010 RGB LEDs (`TZ-H1010` / `XL-1010RGBC-WS2812B-S`) are partitioned across **8 parallel GPIO strings (`GPIOA[7:0]`, `PA0..PA7`)** to achieve a $1,695\text{ Hz}$ refresh rate ($590\,\mu\text{s}$ per 17-LED string):
- **Triangle A (66 LEDs)**: `STR0` (`PA0`, 17 LEDs `D1..D17`), `STR1` (`PA1`, 17 LEDs `D18..D34`), `STR2` (`PA2`, 16 LEDs `D35..D50`), `STR3` (`PA3`, 16 LEDs `D51..D66`).
- **Triangle B (66 LEDs)**: `STR4` (`PA4`, 17 LEDs `D67..D83`), `STR5` (`PA5`, 17 LEDs `D84..D100`), `STR6` (`PA6`, 16 LEDs `D101..D116`), `STR7` (`PA7`, 16 LEDs `D117..D132`).
- Furthermore, adjacent triangles across the jacket tessellation alternate in physical orientation ($\triangle$ upright vs. $\bigtriangledown$ inverted $180^\circ$, plus seam rotations of $\pm 60^\circ / \pm 120^\circ$), and individual strings snake across rows to minimize PCB trace crossings.

Somewhere between the phone's logical `(triangle_id, row, col)` color array and the `GPIOA->ODR` DMA register on the `TriangleNode` MCU, the pixel data must be reshuffled into **physical string & step order** and **`G, R, B` native color byte order**.

### 2.1 Trade-Off Analysis Across the Three Candidate Stages

| Pipeline Stage | How Reshuffling Would Work | Pros | Cons / Bottlenecks | Verdict |
| :--- | :--- | :--- | :--- | :--- |
| **Option A: TriangleNode MCU (`PY32F003F18U6`)** | Phone sends logical row-major triangle order; each `PY32F003` stores an orientation & string permutation table `remap_lut[132]` in Flash and gathers pixels before bit-transposing into `GPIOA->ODR`. | Wire order matches logical triangle row/col. | 1. **CPU Cycle Penalty**: Indirect lookup (`rx_buf[remap_lut[i]]`) inside the $590\,\mu\text{s}$ Delta-Sigma dithering + 8-string bit-transposition loop adds ~2,000 Cortex-M0+ cycles per subframe.<br>2. **Orientation State**: Every node would need to know its physical mounting angle ($\theta \in \{0^\circ, 60^\circ, \dots, 300^\circ\}$) and mirror state.<br>3. **Padding Waste**: Strings `STR2,3,6,7` have 16 LEDs while `STR0,1,4,5` have 17 LEDs; assembling 8-string parallel words on-the-fly requires branchy bounds checks or extra SRAM copy. | **Rejected** |
| **Option B: Central Controller FPGA (`Lattice ECP5`)** | Phone sends logical triangle order; FPGA uses Block RAM lookup tables to permute LEDs before pushing bytes into the 70 UART TX engines. | Offloads node MCU; keeps phone output geometrically simple. | 1. **Massive BRAM Consumption**: Storing arbitrary orientation + string-routing maps for 112,200 LEDs requires $>224\text{ kB}$ of permutation LUTs, exhausting the ECP5's $243\text{ kB}$ internal EBR Block RAM.<br>2. **Breaks Cut-Through Streaming**: FPGA cannot stream bytes directly from the `FT2232H` FIFO to the 70 UARTs; it must wait to random-access reorder within each node block. | **Rejected** |
| **Option C: Phone Companion App (At LUT Build / Frame Pack Time)** | When loading the jacket calibration file (`jacket_topology.json`), the phone pre-computes a single flat gather index table `uint32_t wire_to_canvas_lut[TOTAL_WIRE_SLOTS]`. During frame packing, the phone writes each output byte by indexing directly through this table. | 1. **Zero Runtime Overhead**: The phone *already* gathers colors from the shader/video buffer for each LED. Doing that gather in exact wire-transmission order costs **0 extra CPU/GPU instructions**.<br>2. **Zero Node Indirection**: The `PY32F003` receives pixels already ordered by `(led_step 0..16, string 0..7)` in `G, R, B` order, allowing linear pointer increment straight into the 8-string bit-transposer.<br>3. **Zero FPGA Permutation State**: The FPGA acts as a pure, ultra-fast deterministic packet router. | **SELECTED (Optimal)** |

### 2.2 Exact Pre-Shuffled Intra-Node Memory Layout
Because **Option C (Phone Pre-Shuffling)** is used, the phone packs each node's payload in **8-String Step-Interleaved `GRB` Order** (or contiguous `STR0..STR7` order matching the node's unrolled bit-transposer):

```text
Node Payload Order (132 LEDs, or 136 LEDs with 4 dummy pad slots for 17x8 symmetry):
  Step 0  (First LED of each string):  STR0_LED0(G,R,B), STR1_LED0(G,R,B), ..., STR7_LED0(G,R,B)  [24 Bytes]
  Step 1  (Second LED of each string): STR0_LED1(G,R,B), STR1_LED1(G,R,B), ..., STR7_LED1(G,R,B)  [24 Bytes]
  ...
  Step 15 (16th LED of each string):   STR0_LED15(G,R,B) .. STR7_LED15(G,R,B)                     [24 Bytes]
  Step 16 (17th LED of STR0,1,4,5):    STR0_LED16(G,R,B), STR1_LED16(G,R,B), STR4_LED16(G,R,B), STR5_LED16(G,R,B) [12 Bytes]
  -----------------------------------------------------------------------------------------------------------
  Total Active Payload per Dual-Triangle Node = 396 Bytes (RGB888) or 264 Bytes (RGB565)
```

> [!TIP]
> **Why Step-Interleaved `GRB` Order is Magic for the `PY32F003`**:
> At Step $k \in \{0..16\}$, the 8 strings' Green bytes (`STR0_G .. STR7_G`) sit right next to each other in `rx_back_buf`. The `PY32F003` CPU loads those 8 bytes, runs 16-bit Delta-Sigma dithering, and immediately transposes them into the $8\times 8$ bit-slice matrix for `GPIOA[7:0]` with sequential `ldrb` instructions and **zero cache/bus stride penalties**.

---

## 3. USB Frame Formatting & Packing

### 3.1 Physical USB Interface (`FTDI FT2232H` Channel A)
The Central Controller uses an **`FTDI FT2232H`** High-Speed USB 2.0 (`480 Mbps`) dual-channel transceiver:
- **Channel A (`ADBUS[7:0]` / `ACBUS[7:0]`)**: Configured via external EEPROM (`93LC56B`) into **FT245 Synchronous FIFO Mode**. Driven by a $60\text{ MHz}$ clock output (`CLKOUT`) from the `FT2232H` to the ECP5 FPGA, delivering **$40\text{ MB/s}$ ($320\text{ Mbps}$) sustained bidirectional throughput**.
- **Channel B (`BDBUS[7:0]`)**: Wired to the ECP5 JTAG pins (`TCK`, `TMS`, `TDI`, `TDO`) in MPSSE mode so the phone or a laptop can flash FPGA bitstreams over the exact same USB-C cable using `openFPGALoader`.

### 3.2 Superframe USB Packet Structure (Phone $\rightarrow$ FPGA Bulk-OUT)
Every video frame (or control command) sent from the Phone to the FPGA over the USB Bulk-OUT endpoint is encapsulated in a self-synchronizing **USB Superframe**:

```text
+---------------------------------------------------------------------------------------------------+
|                             USB SUPERFRAME HEADER (16 Bytes)                                      |
+------------+------------+------------+------------+------------+------------+---------------------+
| Byte 0..3  | Byte 4     | Byte 5     | Byte 6     | Byte 7     | Byte 8..11 | Byte 12..15         |
| MAGIC_SYNC | PKT_TYPE   | FRAME_SEQ  | USER_BRIGHT| POLL_CFG   | PAYLOAD_LEN| HEADER_CRC32        |
| 0xAA55F00D | Opcode     | 0..255     | 0..255     | Flags/Chain| uint32_le  | CRC of Bytes 0..11  |
+------------+------------+------------+------------+------------+------------+---------------------+
|                             70-CHAIN INTERLEAVED PAYLOAD (N Bytes)                                |
+---------------------------------------------------------------------------------------------------+
|                             PAYLOAD_CRC32 (4 Bytes, Little-Endian)                                |
+---------------------------------------------------------------------------------------------------+
```

#### Header Field Definitions:
1. **`MAGIC_SYNC` (`4 Bytes`: `0xAA, 0x55, 0xF0, 0x0D`)**: Allows the FPGA parser state machine to immediately re-lock onto frame boundaries if a USB cable glitch or host app restart ever truncates a transfer.
2. **`PKT_TYPE` (`1 Byte`)**:
   - `0x01` (`FMT_RGB565`): **Default High-Speed Video Mode** ($2\text{ B/LED} = 264\text{ B/node}$ for 132 LEDs, or $132\text{ B/node}$ for 66 LEDs). Expanded to 16-bit linear (`Q8.8`) on the `TriangleNode` MCU via hardware-accelerated lookup table.
   - `0x02` (`FMT_RGB888`): **Uncompressed 24-bit TrueColor Video Mode** ($3\text{ B/LED} = 396\text{ B/node}$ for 132 LEDs, or $198\text{ B/node}$ for 66 LEDs).
   - `0x03` (`FMT_LUT8`): **8-bit Indexed Palette Mode** ($1\text{ B/LED} = 132\text{ B/node}$). Ultra-fast mode for 120+ FPS strobe/shader effects.
   - `0x04` (`FMT_SOLID_NODE`): **Solid/Gradient Per-Triangle Mode** ($6\text{ B/node}$: `GRB_TriangleA[3]`, `GRB_TriangleB[3]`).
   - `0x10` (`CMD_PASSTHROUGH`): Direct RS-485 command injection from the phone (used during Commissioning, Aloha Discovery, Visual Chase, and Factory Test Mode).
   - `0x20` (`CFG_TOPOLOGY`): Updates the FPGA's per-chain active node count table (`chain_len[0..69]`) and power ceiling override.
3. **`FRAME_SEQ` (`1 Byte`)**: Rolling $0..255$ frame counter; echoed back in the FPGA's upstream USB telemetry packet so the phone can measure exact round-trip USB pipeline latency.
4. **`USER_BRIGHT` (`1 Byte`)**: User-selected master brightness fader ($0..255$). The FPGA multiplies this by its hardware Automatic Brightness Limiter (`ABL_SCALE`) to produce the final `GLOB_SCALE` byte transmitted to the nodes.
5. **`POLL_CFG` (`1 Byte`)**: Bit 7 = `AUTO_RR_POLL` ($1 =$ FPGA automatically round-robin polls node temperatures; $0 =$ phone specifies explicit chain/slot).

### 3.3 Why the USB Payload is Byte-Interleaved Across All 70 Chains
Inside the Superframe payload, data for the 70 RS-485 chains is packed in **70-Way Byte-Interleaved Order**:

```text
Offset +0..69:    Byte 0 of Chain 0, Byte 0 of Chain 1, ..., Byte 0 of Chain 69
Offset +70..139:  Byte 1 of Chain 0, Byte 1 of Chain 1, ..., Byte 1 of Chain 69
...
Offset +70*k..:   Byte k of Chain 0, Byte k of Chain 1, ..., Byte k of Chain 69
```
*(Chains with fewer nodes than $K_{\max}$ are padded with `0x00` trailing bytes or gated off by the FPGA's `chain_len[c]` register once their `CRC-16` trailer has been transmitted).*

**Why 70-Way Byte Interleaving?**:
1. **Zero Store-and-Forward Latency (Cut-Through Streaming)**: As soon as the first 70 bytes arrive from the `FT2232H` FIFO ($70 / 40\text{ MB/s} = \mathbf{1.75\,\mu\text{s}}$!), the FPGA has Byte 0 for every single one of the 70 UART transmitters and can begin broadcasting on all 70 RS-485 chains simultaneously.
2. **Sequential BRAM Access**: A single linear read pointer in the FPGA's dual-port Block RAM feeds all 70 `ChainTrain_TX` shift registers in one 70-clock-cycle sweep per UART byte time ($3.33\,\mu\text{s}$ at $3.0\text{ Mbps}$, which corresponds to $200$ clock cycles at $60\text{ MHz}$—leaving $130$ cycles of margin per byte!).

---

## 4. Should the Frames Be Compressed? (Overhead vs. Benefit Analysis)

A critical architectural question is whether video frames should be compressed before transmission, and whether the FPGA should contain a hardware decompression engine. We analyze both links in the system:

### 4.1 Link 1: Phone $\rightarrow$ FPGA over USB 2.0 High-Speed (`FT2232H` Synchronous FIFO)

Let us calculate the exact uncompressed bandwidth and transfer time over the $40\text{ MB/s}$ (`FT2232H` Sync FIFO) USB link for the maximum 1,700-triangle ($112,200\text{ LED}$) full-jacket configuration at $60\text{ FPS}$:

| Frame Encoding Mode | Bytes per LED | Frame Size ($112,200\text{ LEDs}$) | Data Rate at $60\text{ FPS}$ | `%` of `FT2232H` $40\text{ MB/s}$ Limit | USB Transfer Time (Cut-Through Overlap) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **`FMT_RGB888` (24-bit Uncompressed)** | $3.0\text{ B}$ | $336.6\text{ kB}$ | $20.20\text{ MB/s}$ | $50.5\%$ | $8.4\text{ ms}$ (100% overlapped with RS-485 TX) |
| **`FMT_RGB565` (16-bit Gamma-Mapped)** | $2.0\text{ B}$ | $224.4\text{ kB}$ | $13.46\text{ MB/s}$ | $33.7\%$ | $5.6\text{ ms}$ (100% overlapped with RS-485 TX) |
| **`FMT_LUT8` (8-bit Palette Indexed)** | $1.0\text{ B}$ | $112.2\text{ kB}$ | $6.73\text{ MB/s}$ | $16.8\%$ | $2.8\text{ ms}$ |
| *(Smaller 3,408–10,000 LED Config)* | $3.0\text{ B}$ | $10.2–30.0\text{ kB}$ | $0.61–1.80\text{ MB/s}$ | $1.5\%–4.5\%$ | $0.25–0.75\text{ ms}$ |

#### Why Entropy Compression (LZ4 / Zstandard / PNG / H.264) on the USB Link is NOT Worth It:
1. **The Bottleneck is RS-485 Wire Time ($3.0–4.0\text{ Mbps}$), NOT the $320\text{ Mbps}$ USB FIFO**:
   - The `FT2232H` Synchronous FIFO moves data at **$320\text{ Mbps}$**, whereas all 70 RS-485 chains combined move data at $70 \times 3.0\text{ Mbps} \times \frac{8}{10} = \mathbf{168\text{ Mbps}}$ payload rate.
   - Because USB is already faster than the aggregate of all 70 RS-485 chains, the USB FIFO easily outruns the RS-485 transmitters **without any compression**.
2. **Entropy Compression Breaks Deterministic Cut-Through Streaming & Power Accumulation**:
   - With uncompressed (`RGB888` or `RGB565`) frames, the FPGA sums frame current for Automatic Brightness Limiting (ABL) and feeds the 70 UARTs directly.
   - Adding variable-length entropy decompression (like LZ4 or Deflate) inside the FPGA would require decoding into an intermediate $336\text{ kB}$ frame buffer (exceeding internal ECP5 BRAM and forcing external SDRAM read/write arbitrations) while adding bit-error vulnerability.
3. **Why Fixed-Ratio Pixel Packing (`FMT_RGB565`) IS Worth It**:
   - Unlike entropy compression, **`FMT_RGB565` ($2\text{ Bytes/LED}$)** is a **fixed-stride representation** that requires **zero FPGA decompression**: the FPGA passes the `RGB565` bytes straight through to the RS-485 chains, cutting **RS-485 wire transmission time by 33%**!
   - On the `TriangleNode` MCU, converting `RGB565` to 16-bit linear `G, R, B` (`Q8.8`) is a single lookup into flash (`gamma5_lut[32]` and `gamma6_lut[64]`), which takes *fewer* Flash bytes ($192\text{ B}$ vs. $512\text{ B}$) and zero extra CPU cycles compared to `RGB888`. Because the `TriangleNode` runs $1,695\text{ Hz}$ Delta-Sigma temporal dithering on the resulting 16-bit linear value, `RGB565` input still produces smooth, stepless 12–14 bit perceptual gradients!

### 4.2 Link 2: Optional Wireless Stage Media Server (Laptop $\rightarrow$ Wi-Fi 6 $\rightarrow$ Phone $\rightarrow$ USB)
What if an external VJ laptop (running Resolume Arena or TouchDesigner) streams live frames wirelessly to the Phone in the wearer's pocket over Wi-Fi 6 (`802.11ax` 5 GHz / 6 GHz), and the Phone forwards them over USB to the Central Controller?

- **Rule 1 — NEVER use lossy video codecs (H.264 / H.265 / VP9)**:
  - Hardware video encoders buffer $1..2$ frames and introduce $8\text{ ms} - 24\text{ ms}$ of encode/decode pipeline latency.
  - Chroma subsampling (`YUV 4:2:0`) smears sharp single-pixel LED chase boundaries across adjacent triangles on the jacket.
- **Rule 2 — Use Raw UDP or Lossless `LZ4` Decompressed on the Phone CPU**:
  - Over a local 5 GHz Wi-Fi 6 link ($600–1,200\text{ Mbps}$ PHY rate), $13.5–20.2\text{ MB/s}$ can be sent via raw UDP or compressed by $3\times–8\times$ down to **$2.5–6.0\text{ MB/s}$ ($20–48\text{ Mbps}$)** using **lossless `LZ4`**.
  - On the Phone's Snapdragon / Tensor ARMv9 core, `LZ4_decompress_fast()` runs at **$>3.5\text{ GB/s}$**—decompressing a $336\text{ kB}$ frame in **$< 0.1\text{ ms}$** ($100\,\mu\text{s}$)!
  - The Phone then writes the **uncompressed** superframe out over USB to the `FT2232H` FIFO.
- **End-to-End Wireless-to-LED Latency Budget**:
  - Laptop Render + `LZ4` Compress ($0.1\text{ ms}$) + Wi-Fi 6 UDP Airtime ($1.8\text{ ms}$) + Phone `LZ4` Decompress ($0.1\text{ ms}$) + USB Cut-Through Stream & RS-485 Latch ($1.5–10\text{ ms}$) = **$< 12\text{ ms}$ total glass-to-photon latency** ($< 1$ video frame at 60 Hz), with **zero decompression logic required in the FPGA**!

---

## 5. Upstream Telemetry Protocol (FPGA $\rightarrow$ Phone Bulk-IN)

While the Phone sends video frames downstream, the FPGA continuously aggregates hardware telemetry and sends a fixed-size **64-byte Upstream Telemetry Packet** back to the Phone over the `FT2232H` Synchronous FIFO IN channel at **$200\text{ Hz}$** (every $5.0\text{ ms}$, synchronized to the IMU sample tick):

```text
+---------------------------------------------------------------------------------------------------+
|                        UPSTREAM USB TELEMETRY PACKET (64 Bytes @ 200 Hz)                          |
+-------------+-------------+-----------------------------------------------------------------------+
| Byte Offset | Field Name  | Description                                                           |
+-------------+-------------+-----------------------------------------------------------------------+
| 0..3        | MAGIC_SYNC  | 0x55, 0xAA, 0x7E, 0x1E (Upstream Telemetry Sync Header)               |
| 4           | PKT_TYPE    | 0x81 = Periodic 200 Hz Telemetry; 0x82 = Aloha Discovery Reply        |
| 5           | LAST_F_SEQ  | Echoes last latched video `FRAME_SEQ` (for RTT latency monitoring)    |
| 6..7        | SYS_STATUS  | Bitflags: [0]=PD_OK, [1]=EXT_5V, [2]=ABL_ACTIVE, [3]=OTP_WARN, [4]=OCP|
| 8..9        | VBUS_MV     | Negotiated USB-PD / Input Voltage in mV (e.g., 48000 = 48.0V)         |
| 10..11      | IBUS_MA     | Negotiated PD Current Ceiling in mA (e.g., 5000 = 5.0A)               |
| 12..13      | BUCK_IMON_MA| Live Measured 5V GaN Buck Output Current in 10 mA units (0..250.0A)   |
| 14          | GLOB_SCALE  | Active Hardware ABL Brightness Scale applied by FPGA (0..255)         |
| 15          | CC_TEMP_C   | Central Controller PCB / GaN Buck Temperature in deg C                |
| 16..23      | IMU0_TORSO  | Central Controller Torso Quaternion (qw, qx, qy, qz as int16 Q14)     |
| 24..31      | IMU1_L_UPP  | Left Upper Arm Quaternion (qw, qx, qy, qz as int16 Q14)               |
| 32..39      | IMU2_L_WRT  | Left Forearm/Wrist Quaternion (qw, qx, qy, qz as int16 Q14)           |
| 40..47      | IMU3_R_UPP  | Right Upper Arm Quaternion (qw, qx, qy, qz as int16 Q14)              |
| 48..55      | IMU4_R_WRT  | Right Forearm/Wrist Quaternion (qw, qx, qy, qz as int16 Q14)          |
| 56          | POLL_CHAIN  | Chain ID (0..69) of the latest round-robin node telemetry sample      |
| 57          | POLL_SLOT   | Node Slot ID (0..26) of the latest round-robin node telemetry sample  |
| 58          | NODE_TEMP   | Reported Node Temperature in 0.5 deg C steps (`temp_c = val*0.5 - 20`)|
| 59          | NODE_VDD    | Reported Node Local +5V Rail in 20 mV steps (`v_mv = 3000 + val*20`)  |
| 60..63      | CRC32       | IEEE 802.3 CRC-32 over Bytes 0..59                                    |
+-------------+-------------+-----------------------------------------------------------------------+
```

- **Bandwidth**: $64\text{ Bytes} \times 200\text{ Hz} = \mathbf{12.8\text{ kB/s}}$ ($0.03\%$ of the $40\text{ MB/s}$ FIFO bandwidth).
- **Sub-Millisecond Flushing**: Whenever the FPGA finishes writing the 64th byte into the `FT2232H` FIFO, it pulses the `SIWU#` (Send Immediate / WakeUp) pin low for 1 clock cycle, forcing the `FT2232H` USB engine to dispatch the USB Bulk-IN packet on the very next $125\,\mu\text{s}$ USB 2.0 High-Speed microframe instead of waiting for the FTDI latency timer!
