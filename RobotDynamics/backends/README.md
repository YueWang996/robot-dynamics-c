# Math backends

A backend is a header that defines `RD_SINCOS` and/or `RD_SQRT`. Point the
library at it and it displaces the portable implementation:

```c
#define RD_MATH_BACKEND "backends/rd_cordic_stm32g4.h"
#include "robot_dynamics.h"
```

They are macros, not function pointers. `rd_sincos()` runs once per revolute
joint inside `rd_update_kinematics()`'s loop, and an indirect call there costs
more than most accelerators save.

| | |
|---|---|
| `RD_SINCOS(x, sp, cp)` | sine and cosine of `x`, into `*(sp)` and `*(cp)` |
| `RD_SQRT(x)` | square root of `x`, as an expression |

Define one and leave the other alone if that is all your part has.

## What ships here

| | |
|---|---|
| [`rd_cordic_stm32g4.h`](rd_cordic_stm32g4.h) | `RD_SINCOS` on the STM32G4 and STM32H7 CORDIC coprocessor. No vendor headers — the three registers are written out in the file. |

## Writing one

Read `rd_cordic_stm32g4.h`; it is short and it is the whole contract. Two
things to get right, both of which cost accuracy silently if you don't:

**Reduce the argument in two steps.** Dividing by π and then folding leaves the
residual carrying only the low bits of a number as large as `x/π`. At `x = 200`
that is an ulp of 3.8e-06 and the answer comes out 1.4e-05 wrong. Subtracting
`k·π` as a high part and a low part keeps the bits.

**Check it before you time it.** In `benchmark/stm32g4`, `make TRIG=1` sweeps
`rd_sincos` against libm in double over one turn and over ±200 radians and
prints the worst error. A backend that is fast and wrong fails there rather
than in somebody's robot.

Accuracy is not free even when it looks free elsewhere: a coarser sin/cos
changes what `rd_chain_build()` computes for a joint offset, and that is a
build-time input to which joints get the axis-aligned congruence. See the
commit that made that classification depend on the model instead.
