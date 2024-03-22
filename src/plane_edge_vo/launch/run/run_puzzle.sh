#!/bin/sh

# traj files are in /home/sun/traj/

#cp traj.txt /home/sun/traj/traj_2.txt
#cp time.txt /home/sun/traj/time_2.txt

mode=traj_puzzle

bin/Soledad -mode ${mode}

cp traj.txt /home/sun/traj/



