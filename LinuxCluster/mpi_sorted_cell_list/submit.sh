#!/bin/bash
#PBS -l nodes=2:ppn=16
# Define the working directory
export MYDIR="/newhome/xf16910"
cd $PBS_O_WORKDIR
#-------------------------------------------------
# Determine which nodes the job has
# been allocated to and create a
# machinefile for mpirun
#-------------------------------------------------
# Don’t change anything below this line
#-------------------------------------------------
# Get the job number
export JOBNO="`echo $PBS_JOBID | sed s/.master.cm.cluster//`"
# Generate mpirun machinefile -------------------
export CONF="$MYDIR/machines.$JOBNO"
for  i in  `cat $PBS_NODEFILE`;
do echo $i >> $CONF
done
# Get the number of processors -----------------
export NUMPROC=`cat $PBS_NODEFILE|wc -l`
# Execute the code ------------------------------
mpirun -machinefile $CONF -np $NUMPROC ./cell_list_sort
