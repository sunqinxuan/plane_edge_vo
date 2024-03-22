#!/bin/sh

seq_path="/home/sun/dataset/rgbd_dataset_freiburg3_long_office_household"
seq_name="fr3_office"
time_interval=0.1
pln_fitting_method=1
alpha=1
beta=1
start_time=0
min_inliers=3000
useWeight=1

vis_every_n_frames=1
delta_time=1
delta_angle=20
#6.7
delta_dist=1.5
key_frame=7

mkdir expr_results/
mkdir expr_results/${seq_name}/
mkdir expr_results/${seq_name}/time_inverval_${time_interval}s/
mkdir expr_results/${seq_name}/time_inverval_${time_interval}s/every_n_frames_${vis_every_n_frames}/
mkdir expr_results/${seq_name}/time_inverval_${time_interval}s/every_n_frames_${vis_every_n_frames}/delta_time_${delta_time}_delta_angle_${delta_angle}_delta_dist_${delta_dist}/
mkdir expr_results/${seq_name}/time_inverval_${time_interval}s/every_n_frames_${vis_every_n_frames}/delta_time_${delta_time}_delta_angle_${delta_angle}_delta_dist_${delta_dist}/beta_${beta}_alpha_${alpha}/

path="expr_results/${seq_name}/time_inverval_${time_interval}s/every_n_frames_${vis_every_n_frames}/delta_time_${delta_time}_delta_angle_${delta_angle}_delta_dist_${delta_dist}/beta_${beta}_alpha_${alpha}/"

evaluate/evaluate_ate.py ${seq_path}/groundtruth.txt traj.txt --plot ${path}/ate_${start_time}_${min_inliers}.png --offset 0 --scale 1 --verbose

evaluate/evaluate_ate.py ${seq_path}/groundtruth.txt traj_opti.txt --plot ${path}/ate_${start_time}_${min_inliers}_opti.png --offset 0 --scale 1 --verbose

#evaluate/evaluate_rpe.py ${seq_path}/groundtruth.txt traj.txt --fixed_delta  --delta 0.1 --delta_unit s --plot ${path}/rpe_${start_time}_${min_inliers}.png --offset 0 --scale 1 --verbose

cp traj.txt ${path}/traj_${start_time}_${min_inliers}.txt
cp traj_opti.txt ${path}/traj_${start_time}_${min_inliers}_opti.txt
cp time.txt ${path}/time_${start_time}_${min_inliers}.txt

gedit ${path}/notes_${start_time}_${min_inliers}.txt


