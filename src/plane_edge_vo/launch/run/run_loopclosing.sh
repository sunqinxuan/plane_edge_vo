#!/bin/sh

#mode=close_loop
mode=close_loop_file

vis_every_n_frames=10
delta_time=3.1
delta_angle=7
delta_dist=0.1
key_frame=10

#seq_path="/media/sun/Elinor.Sun/research/dataset/rgbd_dataset_freiburg2_xyz"
#seq_path="/home/sun/dataset/rgbd_dataset_freiburg1_room"
seq_path="/home/sun/dataset/data316"
seq_name="fr1_room"
time_interval=0.1
pln_fitting_method=1
alpha=1
beta=1
start_time=1510574893
min_inliers=3000
useWeight=1

total_frames=50000
usePln=1
usePt=1

max_icp=10
max_lm=10
occluding=1
curvature=0
canny=0


bin/Soledad -st ${start_time} -ds ${seq_path} -ti ${time_interval} -plnfit ${pln_fitting_method} -vis ${vis_every_n_frames}-pln ${usePln} -pt ${usePt} -mi ${min_inliers} -alpha ${alpha} -beta ${beta} -frames ${total_frames} -icp ${max_icp} -lm ${max_lm} -occluding ${occluding} -curvature ${curvature} -canny ${canny} -useWeight ${useWeight} -mode ${mode} -delta_time ${delta_time} -delta_angle ${delta_angle} -delta_dist ${delta_dist} -key_frame ${key_frame}

#evaluate/evaluate_ate.py ${seq_path}/groundtruth.txt traj.txt --plot ate.png --offset 0 --scale 1 --verbose

#evaluate/evaluate_ate.py ${seq_path}/groundtruth.txt traj_opti.txt --plot ate_opti.png --offset 0 --scale 1 --verbose

mv traj.txt traj11.txt
mv traj_opti.txt traj.txt


