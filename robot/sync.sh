BASEDIR=$(dirname $0)
cd "${BASEDIR}"

source .venv/bin/activate

function install_wheel() {
  pip install --upgrade --find-links=../whl --only-binary :all: $1
}

cat whl-requirements.txt | while read line || [[ -n $line ]];
do
  if ! [[ -z "${line//[[:space:]]/}" ]]; then
    install_wheel $line
  fi
done

robotpy sync --find-links ../whl --use-certifi