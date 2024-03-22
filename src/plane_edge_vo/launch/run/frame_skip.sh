#!/bin/sh

Folder_A="/media/sun/Elements/seq/depth/"

Output_file="/media/sun/Elements/seq/frame_skip.txt"

: > $Output_file

for file_a in `ls ${Folder_A}`
    do
    time=`echo $file_a|cut -c1-8`
    echo "$time $file_a"
    echo "$time depth/$file_a" >> $Output_file
done
