#!/bin/sh

seq_path="/home/sun/dataset/rgbd_dataset_freiburg2_xyz"
seq_name="fr2_xyz"

mkdir expr_results/
mkdir expr_results/${seq_name}/

path="expr_results/${seq_name}/"

evaluate/evaluate_ate.py ${seq_path}/groundtruth.txt traj.txt --plot ${path}/ate.png --offset 0 --scale 1 --verbose

evaluate/evaluate_ate.py ${seq_path}/groundtruth.txt traj_orig.txt --plot ${path}/ate_orig.png --offset 0 --scale 1 --verbose

evaluate/evaluate_ate.py ${seq_path}/groundtruth.txt traj_opti.txt --plot ${path}/ate_opti.png --offset 0 --scale 1 --verbose

cp traj.txt ${path}/traj.txt
cp traj_opti.txt ${path}/traj_opti.txt
cp traj_orig.txt ${path}/traj_orig.txt
cp loop_closure.txt ${path}/loop_closure.txt

gedit ${path}/notes.txt
