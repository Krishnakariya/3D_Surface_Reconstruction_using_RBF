# Computer-Graphics-Course-Project

Requirements: 

Point Cloud Library Installed ( Link: https://github.com/PointCloudLibrary/pcl )

Installing Instructions:
1. Clone the repository. 
2. Go to the source directory.
3. Make a new directory inside source directory named "build".
4. Execute using terminal "cd build".
5. Exectue using terminal "cmake .."

Compiling and Running Instructions
1. Open main.cpp and go to line no. 160 inside loadCloud() function, 
    change "/home/krishna/Desktop/CG/project/hand_gestures/hand_0/image_0000.pcd" to path of the file which has point cloud (.pcd file only allowed).
2. cd build
3. Exectue using terminal "make"
4. Executable "main" would have been generated. Run ./main to see output of the program. 