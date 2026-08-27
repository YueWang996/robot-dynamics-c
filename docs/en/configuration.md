# Compile-time options {#configuration}

Every option is a preprocessor macro with a default in `rd_config.h`. Single-header
users define them on the compiler command line; most have a CMake `option()`.

## All options

| Macro | CMake option | Default | Effect |
|---|---|---|---|
| `RD_USE_SINGLE_PRECISION` | `RD_SINGLE_PRECISION` | 1 | rd_real_t is `float`; 0 gives `double` |
| `RD_FAST_TRIG` | `RD_FAST_TRIG` | 1 | Polynomial `sin`/`cos` instead of libm |
| `RD_ENABLE_ABA` | `RD_ENABLE_ABA` | 1 | Compile the articulated-body algorithm |
| `RD_MATH_BACKEND` | — | undefined | Path to a custom math backend header |
| `RD_MAX_LINKS` | — | 16 | Links an rd_model_t can hold |
| `RD_MAX_JOINTS` | — | 12 | Actuated joints it can hold |
| `RD_USE_CMSIS_DSP` | `RD_CMSIS_DSP` | 0 | `sqrt` from CMSIS-DSP |
| `RD_USE_STATIC_ALLOC` | `RD_STATIC_ALLOC` | 0 | Allocation macros return NULL |
| `RD_DEBUG` | `RD_ENABLE_DEBUG` | 0 | Assertions and logging |
| — | `RD_OPTIMIZE_SIZE` | OFF | `-Os` instead of `-O3 -ffast-math` |

@warning `RD_USE_SINGLE_PRECISION` and `RD_ENABLE_ABA` change struct layout.
Every translation unit in the program must use the same values. Otherwise the
unit that allocated the state buffer and the unit that reads it disagree about
field offsets, which reads wrong data without crashing. The CMake target marks
both `PUBLIC` for this reason.

## RD_USE_SINGLE_PRECISION

| | |
|---|---|
| Default | 1 (`float`) |
| Changes struct layout | Yes |

The FPU on a Cortex-M4F or M33 is single precision only, so a `double` build
falls back to software floating point.

| Precision | Error against Pinocchio |
|---|---|
| float32 | 1.9e-06 |
| float64 | 5.5e-15 |

The float32 error is smaller than encoder resolution and the uncertainty in the
inertia parameters, so the default is appropriate for embedded use. float64 is
for checking results on a host.

## RD_FAST_TRIG

| | |
|---|---|
| Default | 1 (enabled) |
| Changes struct layout | No |

Converting a joint angle to a rotation matrix needs `sin` and `cos`, once per
revolute joint per tick.

| Implementation | Cycles per (sin, cos) | Max error over [-π, π] |
|---|---|---|
| Polynomial (this option) | 169 | 6.6e-08 |
| libm | 521 | 5.9e-08 |

Measured inside rd_update_kinematics() on an STM32G474 @ 170 MHz. Both pass the
Pinocchio comparison at the same tolerances.

On Go2 this option removes about 45% of rd_update_kinematics()'s time.

## RD_ENABLE_ABA

| | |
|---|---|
| Default | 1 (compiled) |
| Changes struct layout | Yes |

ABA is the only algorithm with per-node state of its own: an articulated
inertia, a velocity-product acceleration and the U/D/u triple. It is what sizes
rd_state_t.

| | Per link | 40-link model, float32 |
|---|---|---|
| `RD_ENABLE_ABA=1` | 70 rd_real_t | 11,264 bytes |
| `RD_ENABLE_ABA=0` | 45 rd_real_t | 7,264 bytes |

**Set it to 0 when** forward dynamics always uses
`rd_forward_dynamics(RD_FD_CRBA)`, or when only inverse dynamics is needed.

With it off, rd_aba() and rd_aba_ext() are not declared and
rd_forward_dynamics() returns `RD_ERR_INVALID_INDEX` for `RD_FD_ABA`.

Flash barely changes, because `--gc-sections` already drops ABA in a program
that never calls it. The saving is RAM.

## RD_MAX_LINKS and RD_MAX_JOINTS

| | |
|---|---|
| Default | 16 / 12 |
| Changes struct layout | Yes (the size of rd_model_t) |

rd_model_t holds its link array inline, so these macros set the size of every
model object in the program.

| Robot | Links | Joints |
|---|---|---|
| xarm7 | 8 | 7 |
| spine | 10 | 3 |
| Go2 | 31 | 12 |
| G1 | 40 | 29 |

Fixed links count. Most of Go2's 31 links are feet, sensor mounts and inertial
frames; rd_chain_build() folds them into their nearest moving ancestor, so the
dynamics never traverses them.

## RD_MATH_BACKEND

| | |
|---|---|
| Default | undefined |
| Changes struct layout | No |

Lets an MCU with a math coprocessor, such as the CORDIC on an STM32G4 or H7,
take over the trigonometry or the square root.

### Interface

Define `RD_MATH_BACKEND` as the name of your header. `rd_math.h` includes it
before defining anything. Whichever of these macros your header defines
replaces the library's implementation:

| Macro | Semantics |
|---|---|
| `RD_SINCOS(x, sp, cp)` | Sine and cosine of `x`, written to `*(sp)` and `*(cp)` |
| `RD_SQRT(x)` | Square root of `x`, usable as an expression |

Defining only one is fine; the other keeps the library's implementation.

They are macros rather than function pointers because rd_sincos() runs once per
revolute joint in rd_update_kinematics()'s inner loop and accounts for 45% of
that function on Go2. An indirect call there costs more than most accelerators
save, and it prevents the compiler from keeping the caller's values in
registers across it.

### Three ways to configure it

Replace a single operation:

```c
#define RD_SINCOS(x, sp, cp)  my_sincos((x), (sp), (cp))
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

Use a whole backend file:

```c
#define RD_MATH_BACKEND  "my_cordic.h"
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

From the build system, without touching the source:

```make
CFLAGS += '-DRD_MATH_BACKEND="my_cordic.h"'
```

@warning The setting must reach the translation unit that defines
`RD_IMPLEMENTATION`, because that is where the library is compiled. Setting it
globally from the build system is the reliable way. Another translation unit
without the setting will not break struct layout, but rd_sincos() called
directly from there uses the library's implementation, and the two differ in
the last bits.

### Implementation notes

**Reduce the argument in two steps.** Dividing by π and then folding leaves the
residual carrying only the low bits of a number as large as `x/π`. At `x = 200`
the ulp is 3.8e-06 and the answer comes out 1.4e-05 wrong. Subtract `k·π` as a
high part and a low part.

**Verify accuracy before measuring speed.** In `benchmark/stm32g4`, `make
TRIG=1` sweeps rd_sincos() against libm in double precision over one turn and
over ±200 radians and prints the worst error.

**Watch the effect on fast-path selection.** A coarser sin/cos changes the
joint offset matrix rd_chain_build() computes, and the axis-aligned fast path
in CRBA and ABA is selected from that matrix. This once cost Go2's ABA 14%. The
selection now reads the model data directly.

The library ships no backend. `examples/backends/` has a complete STM32G4
CORDIC implementation to work from.

## RD_USE_CMSIS_DSP

| | |
|---|---|
| Default | 0 |

Takes `sqrt` from CMSIS-DSP. Needs CMSIS-DSP on the include path, and under
CMake also `RD_CMSIS_DSP_INCLUDE_DIR`.

For new projects `RD_MATH_BACKEND` reaches the same place without a vendor
dependency.

## RD_USE_STATIC_ALLOC

| | |
|---|---|
| Default | 0 |

Makes `RD_MALLOC` and `RD_CALLOC` return NULL.

The control loop allocates nothing in any case. The one exception is
rd_chain_build(), which allocates once at startup, so with this option it fails.
A fully heapless build is therefore not available yet.
