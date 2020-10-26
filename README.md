This is the source code webpage of our latest work on dense mapping. 

The high resolution video of our submitted paper is: https://1drv.ms/v/s!ApzRxvwAxXqQmguH6-iBshCx8W_J and https://1drv.ms/v/s!ApzRxvwAxXqQmk-uQ3JGZSYcnEdn

Some steps for running this code:
1. Modified the CUDA arch/gen code in CMakeList.txt such that it is matched with your NVIDIA card. (See https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/)
2. Euroc, KITTI, New_college and the large-scale HKUST campus datasets are tested. Corresponding running files are ros_stereo_euroc.cc, ros_stereo_kitti.cc, ros_stereo_euroc, ros_stereo_newCollege.cc, ros_stereo_hkust.cc. To run them, some modifications on config file names (such as sequnece name, camera calibration files, etc.) in main() are needed.

Known issues:
1. Due to the rapid development of CUDA, some CUDA related functions may not work well and have to be fixed.


The source code is released under GPLv3 license. If you use our code, please cite our paper:

[1] Yonggen Ling and Shaojie Shen, "Real-time Dense Mapping for Online Processing and Navigation", Journal of Field Robotics (JFR), 2019.

For more questions, please contact ylingaa at connect dot ust dot hk .
