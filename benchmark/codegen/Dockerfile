# SPDX-License-Identifier: Apache-2.0
#
# Environment for gen.py. Only needed to regenerate the code Pinocchio emits;
# the benchmark itself builds from the generated C with no Python involved.
#
#   docker build -t rd-codegen .
#   docker run --rm -v "$PWD":/work -v /path/to/bard:/urdf rd-codegen \
#       python3 /work/gen.py --urdf-root /urdf -o /work/generated
FROM condaforge/miniforge3:latest

RUN mamba install -y -c conda-forge \
        pinocchio casadi numpy \
    && mamba clean -afy

WORKDIR /work
