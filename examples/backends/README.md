# Math backends

**None of this is part of the library.** RobotDynamics provides a seam and the
portable implementation behind it. A backend is a header of *yours* that
defines `RD_SINCOS` and/or `RD_SQRT`; pointing `RD_MATH_BACKEND` at it
displaces the portable one:

```c
#define RD_MATH_BACKEND "my_cordic.h"
#include "robot_dynamics.h"
```

`robot_dynamics.h` stays one file with nothing behind it — no vendor headers,
no second download, nothing tied to a part. What is in this directory is a
worked example to read and copy from, not a component to depend on.

| | |
|---|---|
| `RD_SINCOS(x, sp, cp)` | sine and cosine of `x`, into `*(sp)` and `*(cp)` |
| `RD_SQRT(x)` | square root of `x`, as an expression |

Define one and leave the other alone if that is all your part has. They are
macros, not function pointers: `rd_sincos()` runs once per revolute joint
inside `rd_update_kinematics()`'s loop, and an indirect call there costs more
than most accelerators save.

## Three ways in

The hook is a macro, so it works the same whether the library arrives as a tree
or as one file. Nothing is compiled ahead of you — `robot_dynamics.h` is all
source, and the `#include RD_MATH_BACKEND` line survives amalgamation because
it names a macro rather than a literal.

**One operation, no extra file.** The library only ever looks for the two
macros; a backend file is a convenience, not a requirement.

```c
#define RD_SINCOS(x, sp, cp)  my_sincos((x), (sp), (cp))
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

**A whole file**, which is what the CORDIC example is:

```c
#define RD_MATH_BACKEND  "my_cordic.h"
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

**From the build system**, when you would rather not touch the source. The
quoting is what a header name has to survive to reach the preprocessor:

```make
CFLAGS += '-DRD_MATH_BACKEND="my_cordic.h"'
```

One rule, and it is the only one: **whatever you choose has to reach the
translation unit that defines `RD_IMPLEMENTATION`**, because that is where the
library itself is compiled. Setting it project-wide from the build system is
the way not to think about it. A second translation unit that includes the
header without the same setting will not break anything — no structure layout
depends on the backend — but `rd_sincos()` called directly from there would be
the portable one, and the two would disagree in the last bits.

## What is here

| | |
|---|---|
| [`rd_cordic_stm32g4.h`](rd_cordic_stm32g4.h) | `RD_SINCOS` on the STM32G4 and STM32H7 CORDIC coprocessor. No vendor headers — the three registers it needs are written out in the file. |

`benchmark/stm32g4` builds against it: `make CORDIC=1`, and
`make SINGLE=1 CORDIC=1` does the same against `dist/robot_dynamics.h`, which
is how the seam is checked end to end. That build is 448 bytes smaller in
`rd_update_kinematics` and has none of the polynomial's coefficients left in
it.

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

Accuracy is not free even where it looks free: a coarser sin/cos changes what
`rd_chain_build()` computes for a joint offset, and that is a build-time input
to which joints get the axis-aligned congruence in CRBA and ABA. Getting that
wrong cost Go2's ABA 14% before the classification was made to depend on the
model instead.
