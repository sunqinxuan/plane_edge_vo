#!/bin/sh

Folder_A="/media/sun/Elinor.Sun/expr/20171128_corridor/depth"
#rename 's/\_/\./' ${Folder_A}/*.png

Output_file="/media/sun/Elinor.Sun/expr/20171128_corridor/groundtruth.txt"

: > $Output_file

echo "# ground truth trajectory" >> $Output_file
echo "# file: 'rgbd_dataset_freiburg1_desk.bag'" >> $Output_file
echo "# timestamp tx ty tz qx qy qz qw" >> $Output_file


for file_a in `ls ${Folder_A}`
    do
    time=`echo $file_a|cut -c1-14`
    #temp_file=`basename $file_a`
    echo "$time 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000"
    #echo -e "\n"
    echo "$time 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000" >> $Output_file
    #echo -e "\n" >> $Output_file
done
