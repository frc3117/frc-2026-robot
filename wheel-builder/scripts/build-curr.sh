BASEDIR=$(dirname $0)
cd "${BASEDIR}"

function build() {
  pip wheel -w ../../whl --no-deps --no-build-isolation --no-binary :all: $1
}

build ../../frc-ballistic-solver
build ../../robot/frc-3117-tools-python