# Setup

cat >/build/venv/bin/g++ <<'SH'
#!/bin/sh
exec arm-frc2025-linux-gnueabi-g++ "$@"
SH
chmod +x /build/venv/bin/g++

cat >/build/venv/bin/gcc <<'SH'
#!/bin/sh
exec arm-frc2025-linux-gnueabi-gcc "$@"
SH
chmod +x /build/venv/bin/gcc

export PATH="/build/venv/bin:$PATH"

/build/venv/bin/cross-pip install setuptools pybind11

/build/venv/bin/cross-pip wheel /packages/frc-ballistic-solver -w /whl --no-deps --no-build-isolation --no-binary :all:
/build/venv/bin/cross-pip wheel /packages/frc-3117-tools-python -w /whl --no-deps --no-build-isolation --no-binary :all: