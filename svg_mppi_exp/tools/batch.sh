#!/bin/bash
# 여러 조건을 순서대로 실행: batch.sh <dur> <tag:yaml[:key=val,...]> ...
DUR=$1; shift
for spec in "$@"; do
  tag=${spec%%:*}; rest=${spec#*:}; yaml=${rest%%:*}; extra=""
  [ "$rest" != "$yaml" ] && extra=$(echo ${rest#*:} | tr ',' ' ')
  bash $HOME/shared_dir/svg_mppi_exp/tools/run_exp.sh $tag dur=$DUR params=$HOME/shared_dir/svg_mppi_exp/configs/$yaml $extra 2>&1 | tail -5
  echo "-----"
done
