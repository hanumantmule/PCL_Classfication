# Readme
The use of 3D data for plant health monitoring is a recent phenomenon. Since 3D point clouds contain more information than plant 2D images, In this paper,
we compare the performance of different keypoint detectors and local feature descriptors combinations along with different classification algorithms, for the plant growth stage and it’s growth condition classification based on 3D point clouds of the plants.

## Getting Started
Clone this project using below github url: https://github.com/hanumantmule/PCL_Classfication

## 1. Process of PCL installation.
1. Download All-in-One Installer : 
```https://github.com/PointCloudLibrary/pcl/releases```
Including OpenNi
2. Set Environmental Variables and restart computer.
```
name: PCL_ROOT
value: C:\Program Files\PCL 1.11.1

name:Path
C:\Program Files\OpenNI2
C:\Program Files\PCL 1.11.1\bin
C:\Program Files\PCL 1.11.1\3rdParty
C:\Program Files\PCL 1.11.1\3rdParty\VTK\bin
C:\Program Files\OpenNI2\Tools

name: OPENNI2_INCLUDE64
value: C:\Program Files\OpenNI2\Include\

name: OPENNI2_LIB64
value: C:\Program Files\OpenNI2\Lib\

name: OPENNI2_REDIST64
value: C:\Program Files\OpenNI2\Redist\
```
3. Create Folder in whatever place (I used C:/lib/PCLTest drive) and add CMakeLists.txt and .cpp file following the online tutorial instructions: 
```
https://pcl.readthedocs.io/projects/tutorials/en/latest/cloud_viewer.html#cloud-viewer
```                          
Add text from this website into CMakeLists.txt and then add   	cloud_viewer.cpp with the code inside.

![cmake setup](https://github.com/hanumantmule/PCL_Classfication/blob/master/Screenshots/2.png?raw=true)


4. In CMake (https://cmake.org/download/) reference the folder where those two files are and reference empty build file.
Press configure and build.
If the cpp file is named **main.cpp** it **wont work** because it must be **cloud_viewer.cpp** .          
Also do not forget to add code from the tutorial website (pointclouds.org) else the code wont build.   
**Press Configure and then Generate**. Do not forget to set compiler to x64 bit. Compile will create project file in build directory.

![cmake setup](https://github.com/hanumantmule/PCL_Classfication/blob/master/Screenshots/4.png?raw=true)

5. In my case I had some warning messages but it worked:

![cmake](https://github.com/hanumantmule/PCL_Classfication/blob/master/Screenshots/5.png?raw=true)

6. Open Visual Studio project and set to project to cloud_viewer.
Build it.

![cmake](https://github.com/hanumantmule/PCL_Classfication/blob/master/Screenshots/6.png?raw=true)

7. Copy any PCD file in the project folder.
8. Change the name of the file in the cloud_viewer.cpp file and run it.
you will see the visualization of the PCD file.   
Example:
![image](https://github.com/hanumantmule/PCL_Classfication/blob/master/Screenshots/9.png?raw=true)

---------------------------------------
## 2. Local feature aggregation
The library depends on ```scikit-learn``` and all the feature aggregation methods extend the scikit-learn BaseEstimator class.

---------------------
## 3. MATLAB Installation


-------------------------
## How to Run
1. Run sequence:
	```Detector --> Descriptor --> Encoder --> Classifier```
2. For a point cloud file, call respective detector and it will save the output in the current folder named as: 
	```INPUT-FILE-NAME_DETECTOR-NAME.pcd```
3. For the keypoint detected file apply the descriptor and output will be saved in the CSV file named as:
	```INPUT-FILE-NAME_DESCRIPTOR-NAME.csv```
4. In this way apply all the detector-descriptor combination pairs on all the point cloud files and save output in CSV file.
5. Now, to run the encoder, open the python workspace folder. 
   Give the CSV file path as input in the ```FV_TEST.PY``` and ```VLAD_TEST.PY``` file.
6. This will save all the descriptor CSV files into the h5 database file.
7. Open the ```CLASSIFIERS.PY``` file to run the classifiers on the h5 database file.
8. Note the classification accuracy in the table.



## References
[1] https://github.com/PointCloudLibrary/pcl/issues/4462  
[2] https://cmake.org/download/  
[3] https://pcl.readthedocs.io/projects/tutorials/en/latest/cloud_viewer.html#cloud-viewer        
[4] https://github.com/PointCloudLibrary/pcl/releases

## Contributing

Please read [CONTRIBUTING.md](https://github.com/hanumantmule/Email_Classification/blob/main/CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Authors

* **Hanumant Mule** - [Github](https://github.com/hanumantmule/)
* **Namrata Kadam** - [Github](https://github.com/NamrataKadam/)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
