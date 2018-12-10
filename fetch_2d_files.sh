#!/bin/bash

COPY_DIR=~/Desktop
PY_ROOT_DIR=$NAO_HOME/core/python/behaviors/
scp nao@10.202.16.51:/home/nao/2d_asami_data.txt nao@10.202.16.51:/home/nao/state_log.txt $COPY_DIR
