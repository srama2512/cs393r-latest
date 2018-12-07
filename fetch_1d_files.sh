#!/bin/bash

COPY_DIR=./
PY_ROOT_DIR=$NAO_HOME/core/python/behaviors/
scp nao@10.202.16.51:/home/nao/*ogger.txt $COPY_DIR

python $PY_ROOT_DIR/plotting_script.py --load_dir $COPY_DIR --save_dir $COPY_DIR
GT_DIR=$PY_ROOT_DIR/real_model_ds3_da3/
cp $GT_DIR/gtActuator.txt $GT_DIR/gtSensor.txt $COPY_DIR/
python $PY_ROOT_DIR/evaluate1d.py --load_dir $COPY_DIR --save_dir $COPY_DIR --simFlag False

eog -n plot.png &
eog -n eval.png &
