#!/bin/sh

Folder_A="/media/sun/Elinor.Sun/expr/20171026_small_scale/DataSet/dep"
#rename 's/\_/\./' ${Folder_A}/*.png

Output_file="/media/sun/Elinor.Sun/expr/20171026_small_scale/DataSet/depth.txt"

: > $Output_file

echo "# depth maps" >> $Output_file
echo "# file: 'xxx.bag'" >> $Output_file
echo "# timestamp filename" >> $Output_file


for file_a in `ls ${Folder_A}`
    do
    time=`echo $file_a|cut -c1-14`
    #temp_file=`basename $file_a`
    echo "$time $file_a"
    #echo -e "\n"
    echo "$time depth/$file_a" >> $Output_file
    #echo -e "\n" >> $Output_file
done
