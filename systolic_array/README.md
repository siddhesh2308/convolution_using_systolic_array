# Design of an M × N Systolic Array

A parameterized systolic array architecture for accelerating matrix multiplication on FPGA hardware. The design follows a **weight-stationary dataflow**: weights are stored locally inside each Processing Element (PE), while input activations propagate horizontally and partial sums propagate vertically through the array. PEs include zero detection so that a zero-valued activation or weight skips its multiply rather than wasting a MAC cycle.

The accelerator is integrated with an FPGA-based UART link for loading operands and streaming back results, and is written entirely in Verilog HDL.

```
Output = Activation × Weight
```

Default size: **3×3**, data width **24-bit signed**.

---

## Working
**Setup**

```
A (input)        B (weights)
[ 1  2 ]         [ 5  6 ]
[ 3  4 ]         [ 7  8 ]
```
Goal: `C = A × B`, computed on a 2×2 grid of 4 PEs.

### Step 1 — Weight loading

`B`'s elements are latched into their PEs via `load_weight` and stay stationary for the whole computation:

| | Col 0 | Col 1 |
|---|---|---|
| **Row 0** | PE(0,0) = 5 | PE(0,1) = 6 |
| **Row 1** | PE(1,0) = 7 | PE(1,1) = 8 |

### Step 2 — Row 0 of A streams through: `A₀ = [1, 2]`

Activation `1` enters PE row 0 at cycle 1; activation `2` enters PE row 1 one cycle later — the skew that lets activations and partial sums meet at the right PE at the right time. Each PE computes `P_out = P_in + (A_in × W)`.

**Cycle 1** — only PE(0,0) fires:

| | Col 0 | Col 1 |
|---|---|---|
| **Row 0** | 0 + (1×5) = **5** | waiting |
| **Row 1** | waiting | waiting |

`5` propagates right (as the activation) and down (as the partial sum).

**Cycle 2** — two PEs fire at once:

| | Col 0 | Col 1 |
|---|---|---|
| **Row 0** | — | 0 + (1×6) = **6** |
| **Row 1** | 5 + (2×7) = **19** | waiting |

**Cycle 3** — the last PE fires:

| | Col 0 | Col 1 |
|---|---|---|
| **Row 0** | — | — |
| **Row 1** | — | 6 + (2×8) = **22** |

First output row complete: `C₀ = [19, 22]`. Notice the four PEs don't all fire at once — the result ripples through as a **diagonal wavefront**, one or two PEs active per cycle.

### Step 3 — Row 1 of A streams through: `A₁ = [3, 4]`

Once the pipeline has drained and partial sums reset (weights stay put), the same skewed pattern repeats:

**Cycle 5**

| | Col 0 | Col 1 |
|---|---|---|
| **Row 0** | 0 + (3×5) = **15** | waiting |
| **Row 1** | waiting | waiting |

**Cycle 6**

| | Col 0 | Col 1 |
|---|---|---|
| **Row 0** | — | 0 + (3×6) = **18** |
| **Row 1** | 15 + (4×7) = **43** | waiting |

**Cycle 7**

| | Col 0 | Col 1 |
|---|---|---|
| **Row 0** | — | — |
| **Row 1** | — | 18 + (4×8) = **50** |

Second output row complete: `C₁ = [43, 50]`.

### Result

```
C = A × B =
[ 19  22 ]
[ 43  50 ]
```
Weights are parallel-loaded into every PE in one cycle (not shifted PE-to-PE); activations stream rightward across a row; partial sums accumulate downward through a column and land in the output (column) buffers.

| Module | Description |
|---|---|
| `PE.v` | Processing Element — MAC, local weight register, zero detection. |
| `Systolic_Array.v` | Instantiates and interconnects the PEs into the ROWS×COLS grid. |
| `Buffer.v` | Generic synchronous storage — used for the weight buffer, row (activation) buffers, and output (column) buffers. |
| `Controller.v` | FSM sequencing the overall computation: `IDLE → LOAD → COMPUTE → STORE → DONE`. |
| `uart_loader.v` | Loads input data from the host into the buffers via UART. |
| `uart_rx.v` | Receives serial data from the host. |
| `uart_tx.v` | Transmits serial data to the host. |
| `result_sender.v` | Streams computed results back to the host. |
| `Top_FPGA.v` | FPGA top-level — integrates all of the above. |

---


## How to Test

**On the FPGA board:**
Generate a bitstream from the RTL using Xilinx Vivado and program the board. Then, on the host PC, run `send_data_new.py`— it sends the input (weight + activation) matrices to the board over UART, and the computed result is sent back over the same link and printed in the terminal.

**In RTL simulation:**
`tb_mxn.sv` drives the design directly and lets you view the waveform. If you just want to verify the compute core without the UART path, you only need: `PE.v`, `Systolic_Array.v`, `Top_Stage2_MxN.v`, `Controller.v`, and `Buffer.v`.


> **Checking results by hand:** the array computes the result as **Aᵀ × Weight**, not A × Weight — transpose the activation matrix before multiplying if you're verifying the output on a calculator.

> **UART data format:** both the input and output matrices are flattened into a single stream of words when sent/received over UART, with each word spaced `WIDTH` bits apart (default: 24 bits).
