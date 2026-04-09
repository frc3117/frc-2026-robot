BASEDIR=$(dirname $0)
cd "${BASEDIR}"

source .venv/bin/activate

sh frc-31117-tools-python/build.sh
robotpy installer download --find-links ../whl -r whl-requirements.txt
read -p "Press any key when connected to robot."

robotpy installer install --find-links ../whl --force-reinstall -r whl-requirements.txt