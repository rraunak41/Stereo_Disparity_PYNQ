# Stereo_Disparity_PYNQ
#  Hardware-Accelerated Stereo Disparity Pipeline on PYNQ-Z2

A high-performance, real-time stereo vision depth-mapping co-processor accelerated on the Zynq-7000 SoC FPGA fabric using Vivado, Vitis HLS, and the PYNQ framework.

---

##  Architectural Overview

To maximize data throughput and minimize DMA overhead, this pipeline packs Left and Right camera frames side-by-side into a single $1280 \times 480$ buffer before streaming them to the programmable logic (PL).

[ DRAM Input Buffer ]
                       │ (1280 x 480 packed pixels)
                       ▼
           [ AXI DMA (MM2S Transmit) ] 
                       │
                       ▼
           [ Verilog Stereo Splitter ] 
          (Demuxes stream into two phases)
           │                             │
           ▼ (Left 640 Phase)            ▼ (Right 640 Phase)
     [axis_data_fifo_0]            [axis_data_fifo_1]
     (Elastic buffer)              (Elastic buffer)
           │                             │
           ▼                             ▼
   [Left Census Core]            [Right Census Core]
(Window signature loop)       (Window signature loop)
           │                             │
           └──────────────┬──────────────┘
                          ▼
              [ HW Disparity Matcher ]
              (Parallel cost calculator)
                          │
                          ▼ (640 x 480 Disparity Stream)
           [ AXI DMA (S2MM Receive) ]
                          │
                          ▼
             [ DRAM Output Buffer ]

### ⚙️ Core Hardware Modules
* **Custom Verilog Splitter FSM:** Efficiently demultiplexes the continuous 1280-pixel stream line-by-line into independent Left (640) and Right (640) processing channels.
* **AXI-Stream FIFOs:** Act as elastic timing shock absorbers to cleanly absorb channel backpressure during spatial window row resets.
* **Dual HLS Census Processors:** 32-bit parallel Vitis HLS cores transforming pixel intensities into spatial bit-signatures for illumination-invariant matching.
* **Hardware Disparity Matcher:** Executes multi-element parallel Hamming-distance window cost matrix calculations to output real-time disparity depths.

---

## 🛠️ Key Engineering Challenges Solved

1. **AXI Address Space Ghosting Defect:** Resolved an unmapped high-speed memory path bottleneck in the Vivado Address Editor that caused AXI crossbar mirroring and register collision lockups.
2. **24-bit Bus Alignment Mismatch:** Stripped out unstable `axis_dwidth_converter` blocks that were causing data length discrepancies (`TKEEP` flags panic). Upgraded the entire fabric layout to a uniform, clean 32-bit width across all interfaces.
3. **Sliding-Window Border Deadlock:** Overcame Jupyter notebook thread freezes caused by sliding-window border pixel clipping. Implemented a production-grade **Status-Driven Driver Polling Loop** in Python that monitors the core's control register directly for an `IDLE` state (`0x04`) combined with channel soft-resets (`stop()` / `start()`).

---

## 📊 Empirical Performance & Validation

The final compiled 32-bit architecture was validated natively on the physical PYNQ-Z2 board under a high-texture noise stress test:

| Architectural Metric | Captured Hardware Verification Value |
| :--- | :--- |
| **Operational Core Span** | **Elements 641 through 307,199** |
| **Active Output Density Yield** | **274,690 / 307,200 pixels (89.42%)** |
| **Computed Depth Resets** | **15 Distinct Disparity Resolution Levels** |
| **System Software Stability** | **🟢 100% Stable (Zero CPU Thread Freezes)** |

> 📌 **Architectural Proof:** Starting at exactly element index `641` mathematically proves the hardware loop is tightly synchronized down to a single clock cycle—perfectly accounting for exactly 1 row of initial hardware line-buffer latency ($640 + 1$) before streaming computed tokens back to DDR.
