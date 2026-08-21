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

The generated sources are 423 KB of C for the three robots, which builds to
about 185 KB of Cortex-M4 code. It fits a 512 KB part; it does not fit
everywhere, and that is part of what the comparison is about.
