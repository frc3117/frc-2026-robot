BASEDIR=$(dirname $0)
cd "${BASEDIR}"

pip wheel --no-deps -w build .

rm -R ./build/lib
rm -R ./build/bdist.*
rm -R *.egg-info