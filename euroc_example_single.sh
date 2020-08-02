#!/bin/bash
pathDatasetEuroc='/home/libing/dataset/euroc' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
echo "Launching MH01 with Monocular sensor"
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml "$pathDatasetEuroc"/MH01_easy ./Examples/Monocular/EuRoC_TimeStamps/MH01_easy.txt dataset-MH01_mono

