# Configuration {#configuration}

Every option is a preprocessor macro with a default in `rd_config.h`, so a
single-header user sets them on the compiler command line and a CMake user
gets an `option()` for most of them.

## The options

| Macro | CMake | Default | What it does |
|---|---|---|---|
| RD_USE_SINGLE_PRECISION | `RD_SINGLE_PRECISION` | 1 | `float` for rd_real_t. `0` gives `double` |
| RD_FAST_TRIG | `RD_FAST_TRIG` | 1 | Polynomial `sin`/`cos` rather than libm's |
| RD_ENABLE_ABA | `RD_ENABLE_ABA` | 1 | Compile the articulated-body algorithm |
| RD_MATH_BACKEND | — | unset | A header of yours defining `RD_SINCOS` and/or `RD_SQRT` |
| RD_MAX_LINKS | — | 16 | Links an rd_model_t can hold |
| RD_MAX_JOINTS | — | 12 | Actuated joints it can hold |
| RD_USE_CMSIS_DSP | `RD_CMSIS_DSP` | 0 | `sqrt` from CMSIS-DSP; wants `RD_CMSIS_DSP_INCLUDE_DIR` |
| RD_USE_STATIC_ALLOC | `RD_STATIC_ALLOC` | 0 | Make the allocation macros return NULL |
| `RD_DEBUG` | `RD_ENABLE_DEBUG` | 0 | Assertions and log output |
| — | `RD_OPTIMIZE_SIZE` | OFF | `-Os` instead of `-O3 -ffast-math` |

@warning RD_USE_SINGLE_PRECISION and RD_ENABLE_ABA change struct layout. Every
translation unit in the program has to be built with the same values, or the
one that allocated the state buffer and the one that reads it will disagree
about where the fields are. The CMake target marks both `PUBLIC` for that
reason.

## Precision

The FPU on a Cortex-M4F or an M33 is single precision and nothing else, so a
double build does its arithmetic in software on exactly the parts this library
was written for.

Single precision agrees with Pinocchio to 1.9e-06 across the test models. That
is finer than the encoder resolution and the inertia numbers feeding it, so it
is the default. Double reaches 5.5e-15 and belongs in a host build that is
checking something.

## Fast trigonometry

RD_FAST_TRIG replaces libm's `sin` and `cos` with a polynomial pair: 169 cycles
per `(sin, cos)` against libm's 521, measured inside rd_update_kinematics() on
an STM32G474 at 170 MHz. Accuracy is 6.6e-08 worst case against double over
[-π, π], where libm is 5.9e-08 — the same, for practical purposes, and both
pass the Pinocchio comparison at the same tolerances.

It is worth 45% of rd_update_kinematics() on Go2, which is the single largest
thing on this page.

## Dropping ABA

ABA is the only algorithm here that carries per-node state of its own: an
articulated inertia, a velocity-product acceleration, and the U/D/u triple its
outward pass needs. That makes it what sizes rd_state_t.

| | floats per link |
|---|---|
| `RD_ENABLE_ABA=1` | 70 |
| `RD_ENABLE_ABA=0` | 45 |

On a 40-link model in a float32 build that is 11,264 bytes against 7,264. Set
it to 0 in a build whose forward dynamics comes from
`rd_forward_dynamics(RD_FD_CRBA)`, or in one that only needs inverse dynamics.
rd_aba() and rd_aba_ext() are then not declared, and rd_forward_dynamics()
answers `RD_ERR_INVALID_INDEX` to `RD_FD_ABA`.

Flash barely moves either way, because `--gc-sections` already drops ABA in a
program that never calls it. The RAM is the reason.

## Model size

RD_MAX_LINKS and RD_MAX_JOINTS bound the arrays inside rd_model_t, and that
struct carries them inline, so they set the size of every model object in the
program whether the robot fills it or not.

| Robot | links | joints |
|---|---|---|
| xarm7 | 8 | 7 |
| spine | 10 | 3 |
| Go2 | 31 | 12 |
| G1 | 40 | 29 |

Fixed links count as links. Most of Go2's 31 are feet, sensor mounts and
inertial frames, and rd_chain_build() folds them into their nearest moving
ancestor so the dynamics never walks them.

## Bringing your own sin/cos

RD_MATH_BACKEND names a header of *yours*. `rd_math.h` includes it before it
defines anything, and whichever of the two macros it defines displaces the
portable implementation:

| | |
|---|---|
| `RD_SINCOS(x, sp, cp)` | sine and cosine of `x`, into `*(sp)` and `*(cp)` |
| `RD_SQRT(x)` | square root of `x`, as an expression |

Define one and leave the other alone if that is all your part has.

They are macros rather than function pointers on purpose. rd_sincos() runs once
per revolute joint inside rd_update_kinematics()'s loop and is 45% of that
function on Go2; an indirect call there costs more than most accelerators save,
and it would stop the compiler keeping the caller's values in registers across
it.

Nothing of the sort ships with the library. `robot_dynamics.h` stays one file
with no vendor headers behind it, and `examples/backends/` in the repository
has a worked one for the STM32G4 CORDIC to read and copy from.

### Three ways in

The hook is a macro, so it works the same whether the library arrives as a tree
or as one file. Nothing is compiled ahead of you, and the
`#include RD_MATH_BACKEND` line survives amalgamation because it names a macro
rather than a literal.

One operation, no extra file:

```c
#define RD_SINCOS(x, sp, cp)  my_sincos((x), (sp), (cp))
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

A whole file, which is what the CORDIC example is:

```c
#define RD_MATH_BACKEND  "my_cordic.h"
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

From the build system, when you would rather not touch the source. The quoting
is what a header name has to survive to reach the preprocessor:

```make
CFLAGS += '-DRD_MATH_BACKEND="my_cordic.h"'
```

@warning One rule, and it is the only one: whatever you choose has to reach the
translation unit that defines `RD_IMPLEMENTATION`, because that is where the
library itself is compiled. Setting it project-wide from the build system is
the way not to think about it. A second translation unit that includes the
header without the same setting breaks nothing — no structure layout depends on
the backend — but rd_sincos() called directly from there would be the portable
one, and the two would disagree in the last bits.

### Writing one

Two things to get right, both of which cost accuracy silently if you do not.

**Reduce the argument in two steps.** Dividing by π and then folding leaves the
residual carrying only the low bits of a number as large as `x/π`. At `x = 200`
that is an ulp of 3.8e-06 and the answer comes out 1.4e-05 wrong. Subtracting
`k·π` as a high part and a low part keeps the bits.

**Check it before you time it.** In `benchmark/stm32g4`, `make TRIG=1` sweeps
rd_sincos() against libm in double over one turn and over ±200 radians and
prints the worst error. A backend that is fast and wrong fails there rather
than in somebody's robot.

Accuracy is not free even where it looks free. A coarser sin/cos changes what
rd_chain_build() computes for a joint offset, and that is a build-time input to
which joints get the axis-aligned congruence in CRBA and ABA. Getting it wrong
cost Go2's ABA 14% before the classification was made to depend on the model
instead of on the computed transform.
