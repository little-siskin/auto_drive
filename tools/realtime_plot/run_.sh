DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export PYTHONPATH="${DIR}/../../../bazel-genfiles:${PYTHONPATH}"
eval "python ${DIR}/realtime_plot.py $1"