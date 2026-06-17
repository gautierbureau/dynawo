#!/bin/bash

export DYNAWO_HOME=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Precompiled OpenModelica (patched for Dynawo) extracted from the Dynawo
# v1.7.0 Linux release. The precompiled bundle has bin/include/lib directly
# under OpenModelica/, so SRC and INSTALL point to the same directory.
export DYNAWO_SRC_OPENMODELICA=$DYNAWO_HOME/OpenModelica
export DYNAWO_INSTALL_OPENMODELICA=$DYNAWO_HOME/OpenModelica

# liblpsolve55.so ships in a non-default directory on Ubuntu.
export LD_LIBRARY_PATH=/usr/lib/lp_solve:$LD_LIBRARY_PATH

export DYNAWO_LOCALE=en_GB
export DYNAWO_RESULTS_SHOW=true
export DYNAWO_BROWSER=firefox

export DYNAWO_NB_PROCESSORS_USED=1

export DYNAWO_BUILD_TYPE=Release

$DYNAWO_HOME/util/envDynawo.sh $@
