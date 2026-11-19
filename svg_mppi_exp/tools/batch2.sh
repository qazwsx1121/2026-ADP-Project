#!/bin/bash
# batch2.sh <dur> "<tag> <run_exp key=val ...>" ...
DUR=$1; shift
for spec in "$@"; do
  set -- $spec
  tag=$1; shift
  bash $HOME/shared_dir/svg_mppi_exp/tools/run_exp.sh $tag dur=$DUR "$@" 2>&1 | tail -5
  echo "-----"
done
