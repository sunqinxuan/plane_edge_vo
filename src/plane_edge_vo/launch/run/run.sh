#!/bin/sh

mode=debug
#mode=vis_scan

#seq_path="/home/sun/dataset/data1206"
#seq_path="/home/zgz/sun/data1206"
seq_path="/home/zgz/sun/RazorEdge/build/data"

time_interval=0.2
pln_fitting_method=1
alpha=1
beta=1
start_time=0

min_inliers=10000
useWeight=1
kinect2=1

vis_every_n_frames=10
#delta_time=7
#delta_angle=10
#delta_dist=0.37
#key_frame=10

total_frames=100
usePln=1
usePt=1

max_icp=10
max_lm=10
occluding=1
curvature=0
canny=0


bin/RazorEdge -st ${start_time} -ds ${seq_path} -ti ${time_interval} -plnfit ${pln_fitting_method} -vis ${vis_every_n_frames}-pln ${usePln} -pt ${usePt} -mi ${min_inliers} -alpha ${alpha} -beta ${beta} -frames ${total_frames} -icp ${max_icp} -lm ${max_lm} -occluding ${occluding} -curvature ${curvature} -canny ${canny} -useWeight ${useWeight} -mode ${mode} -kinect2 ${kinect2}
#-delta_time ${delta_time} -delta_angle ${delta_angle} -delta_dist ${delta_dist} -key_frame ${key_frame}

#evaluate/evaluate_ate.py ${seq_path}/groundtruth.txt traj.txt --plot ate.png --offset 0 --scale 1 --verbose

#evaluate/evaluate_ate.py ${seq_path}/groundtruth.txt traj_opti.txt --plot ate_opti.png --offset 0 --scale 1 --verbose



