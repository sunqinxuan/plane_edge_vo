#!/bin/sh

#mode=online
mode=collect

pln_fitting_method=1
alpha=1
beta=1
thres_weight=0.01
min_inliers=7000
useWeight=1

total_frames=20
usePln=1
usePt=1

max_icp=10
max_lm=10
occluding=1
curvature=0
canny=0

mkdir data
mkdir data/depth
mkdir data/rgb
#mkdir data/reg

rm data/depth/*.png
rm data/rgb/*.png
#rm data/reg/*.png
rm data/*.txt

bin/RazorEdge -plnfit ${pln_fitting_method} -pln ${usePln} -pt ${usePt} -mi ${min_inliers} -alpha ${alpha} -beta ${beta} -frames ${total_frames} -icp ${max_icp} -lm ${max_lm} -occluding ${occluding} -curvature ${curvature} -canny ${canny} -useWeight ${useWeight} -mode ${mode} -thres_weight ${thres_weight}

cp traj.txt data/traj.txt
#mv data data2
