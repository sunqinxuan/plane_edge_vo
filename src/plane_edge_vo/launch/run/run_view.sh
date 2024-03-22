#!/bin/sh

mode=view

#seq_path="/media/sun/Elinor.Sun/expr/20171128_corridor"
seq_path="/home/zgz/sun/RazorEdge/build/data"
traj_path="/home/zgz/sun/RazorEdge/build/data"
key_frame=1
start_time=0
kinect2=1

bin/RazorEdge -mode ${mode} -key_frame ${key_frame} -ds ${seq_path} -st ${start_time} -kinect2 ${kinect2} -traj_path ${traj_path}


