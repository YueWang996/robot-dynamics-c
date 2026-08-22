# Against Pinocchio's code generation

The fastest thing a general-purpose dynamics library can hand you is not its
own recursion but the code it generates for your specific robot: Pinocchio can
trace RNEA, ABA and CRBA symbolically and emit straight-line C with no loops,
no model traversal and every operation spelled out. That is the right thing to
measure against, so this directory does.

```
gen.py             traces the three algorithms with pinocchio.casadi and emits
                   the generated C, plus cg_data.h -- one configuration per
                   robot in both index conventions and reference outputs that
                   Pinocchio computed in double precision
bench_codegen.c    the head-to-head, on the same clock and protocol as the
                   rest of the benchmark
Dockerfile         a reproducible environment for gen.py
generated/         gen.py's output; not in the repository (423 KB of C)
```

Both sides are float32, both `-O3`, same compiler, same board, and **both are
checked against Pinocchio's own double-precision answer** rather than against
each other — they agree with it to about 1e-7, which is float32's resolution.

## What is compared

The generated functions take a configuration and return a result, so they redo
the kinematics on every call. The comparison is therefore
`rd_update_kinematics()` plus the algorithm against one generated call. There
is also a `rnea+crba` row: a whole-body controller wants both, and there our
shared state is computed once while the generated code repeats its kinematics
in each function.

## Running it

```bash
# needs pinocchio with the casadi bindings
conda install -c conda-forge pinocchio casadi      # or use the Dockerfile
python3 gen.py --urdf-root /path/to/bard

# host
gcc -O3 -std=c11 -DRD_MAX_LINKS=40 -DRD_MAX_JOINTS=24 \
    -I ../../RobotDynamics -I .. -I generated \
    bench_codegen.c generated/*.c ../../RobotDynamics/rd_*.c -lm -o bench_cg
./bench_cg

# STM32G474
cd ../stm32g4 && make CODEGEN=1 && make flash
openocd -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg \
  -c init -c "reset halt" -c "arm semihosting enable" -c resume
```

The generated sources are 456 KB of C for the three robots, which builds to
113,667 bytes of Cortex-M4 code -- 17,425 for spine, 29,021 for xarm7, 67,221
for go2 -- against 40,236 bytes for the whole library, which handles any robot.
Those are `.text` of the compiled objects on both sides, same compiler and
flags, nothing removed.

The G474 has 512 KB of flash and takes all three at once. The STM32L413 port
has 128 KB to link into, so generate the subset that fits --
`gen.py --robots go2` -- and bench_codegen.c drops the rest automatically; it
keys off the `CG_<ROBOT>_NQ` macros cg_data.h defines.

Results: [`../results/codegen_stm32g474.csv`](../results/codegen_stm32g474.csv)
for all three robots and
[`../results/codegen_stm32l413.csv`](../results/codegen_stm32l413.csv) for go2,
summarised in the top-level README. Short version: code generation wins on a
desktop by 1.1-1.7x and loses on a Cortex-M4 by 1.5-5.0x, with the gap widening
as the robot grows.
