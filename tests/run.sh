#!/bin/bash

names=('mh1' 'mh2' 'mh3' 'mh4' 'mh5' 'v11' 'v12' 'v13' 'v21' 'v22' 'v23')
starts=(0 0 0 0 0 0 0 0 0 0 0)

# for i in {0..10}
# do
#   for r in {1..10}
#   do
#   echo ${names[i]}_${r}
#   roslaunch dso_ros euroc.launch bag:=$HOME/Workspace/data/EuRoC/${names[i]}.bag \
#     results:=$HOME/Workspace/spline_vio_tests/results/DSO/${names[i]}_${r}.txt
#   done
# done

for i in {5..10}
# for i in {0..10}
do
  for r in {1..10}
  do
  echo ${names[i]}_${r} ${starts[i]}
  roslaunch spline_vio euroc.launch bag:=$HOME/Workspace/data/EuRoC/${names[i]}.bag \
    start_frame:=${starts[i]} \
    results:=$HOME/Workspace/spline_vio_tests/results/SplineVIO/${names[i]}_${r}.txt
  done
done


# for i in {0..10}
# do
#   for r in {1..10}
#   do
#   echo ${names[i]}_${r}
#   roslaunch vi_dso euroc.launch bag:=$HOME/Workspace/data/EuRoC/${names[i]}.bag \
#     results:=$HOME/Workspace/spline_vio_tests/results/VI_DSO/${names[i]}_${r}.txt
#   done
# done


names=('r1' 'r2' 'r3' 'r4' 'r5' 'r6')


# for i in {0..5}
# do
#   for r in {1..10}
#   do
#   echo ${names[i]}_${r}
#   roslaunch dso_ros tum.launch bag:=$HOME/Workspace/data/TUM/${names[i]}.bag \
#     results:=$HOME/Workspace/spline_vio_tests/results/DSO/${names[i]}_${r}.txt
#   done
# done


# for i in {0..5}
# do
#   for r in {1..10}
#   do
#   echo ${names[i]}_${r}
#   roslaunch spline_vio tum.launch bag:=$HOME/Workspace/data/TUM/${names[i]}.bag \
#     results:=$HOME/Workspace/spline_vio_tests/results/SplineVIO/${names[i]}_${r}.txt
#   done
# done


# for i in {0..5}
# do
#   for r in {1..10}
#   do
#   echo ${names[i]}_${r}
#   roslaunch vi_dso tum.launch bag:=$HOME/Workspace/data/TUM/${names[i]}.bag \
#     results:=$HOME/Workspace/spline_vio_tests/results/VI_DSO/${names[i]}_${r}.txt
#   done
# done
