#!/bin/bash

#PBS -N boid_mpi_cell_list
#PBS -o output
#PBS -l nodes=1:ppn=10,walltime=00:20:00
#PBS -m abe

# load the VTune module
module add intel-cluster-studio/vtune/vtune-2015

#! change the working directory (default is home directory)
cd $PBS_O_WORKDIR

echo Running on host `hostname`
echo Time is `date`
echo Directory is `pwd`
echo PBS job ID is $PBS_JOBID
echo This jobs runs on the following machines:
echo `cat $PBS_NODEFILE | uniq`
nnodes=`wc $PBS_NODEFILE | awk '{ print $1 }'`
mpiexec -np $nnodes amplxe-cl -quiet -collect hotspots -result-dir 2milprocess ./cell_list_sort
