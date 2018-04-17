# 568-Final-Project

This is Team 18's final project git repository for EECS 568: Mobile Robotics. The title of our project is Visual Lidar Odometry and Mapping with KITTI, and team members include: Ali Abdallah, Alexander Crean, Mohamad Farhat, Alexander Groh, Steven Liu and Christopher Wernette.

A sample ROS bag file, cut from sequence 08 of KITTI, is provided here:[https://drive.google.com/open?id=1r7nlpAfTL3p1pqSlM7zTQDmk9sHpClZ_]

You can see the results of the algorithm running here: 

<a href="http://www.youtube.com/watch?feature=player_embedded&v=YTCnmP5RBuE
" target="_blank"><img src="http://img.youtube.com/vi/YTCnmP5RBuE/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="560" height="400" border="10" /></a>

## Getting Started

First, we recommend you read through our paper uploaded on this repository. Next, read the three directly related works: VLOAM, LOAM, and DEMO.

We recommend you read through the original [V-LOAM](http://www.frc.ri.cmu.edu/~jizhang03/Publications/ICRA_2015.pdf) paper by Ji Zhang and Sanjiv Singh as a primer. Follow that up with the [LOAM](https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf) paper by the same authors. Finally, conclude with reading [DEMO](http://www.frc.ri.cmu.edu/~jizhang03/Publications/IROS_2014.pdf) paper by Ji Zhang et all. These will give you theoretical understanding of the V-LOAM algorithm, and all three provide many references for further reading.

### Prerequisites

First you will need to install [Ubuntu 16.04](http://releases.ubuntu.com/16.04.4/) in order to run ROS-Kinetic. We recommend primary or dual booting Ubuntu as we encountered many issues using virtual machines, which are discussed in detail in our final paper.
Next up, you will need to install [ROS-Kinetic](http://wiki.ros.org/kinetic) as our algorithm has only been validated on this version of ROS. You can find a detailed installation guide [here](http://wiki.ros.org/kinetic/Installation).
After that step, you will need to download some [KITTI Raw Data](http://www.cvlibs.net/datasets/kitti/raw_data.php). We recommend reading through their [odometry eval kit](http://kitti.is.tue.mpg.de/kitti/devkit_odometry.zip) to decide which Sequence you would like to run.
Following this, you will need to download and install the [kitti2bag utility](https://github.com/tomas789/kitti2bag). Detailed instructions can be found within the github README.md. Convert your KITTI raw data to a ROS .bag file and leave it in your `~/Downloads` directory.
As a final prerequisite, you will need to have Matlab installed to run our benchmarking code, although it is not necessary in order

### Installing
Before installing this package, ensure that velodyne drivers are installed. A ROS package is provided at [https://github.com/ros-drivers/velodyne]. Short summary of installation instructions:
```
$ sudo apt-get install ros-PACKAGE-velodyne  
$ cd ~/catkin_ws/src/  
$ git clone https://github.com/ros-drivers/velodyne.git  
$ cd ~/catkin_ws  
$ catkin_make  
```

After installing velodyne drivers, proceed by cloning our loam_velodyne directory into your `~/catkin/src` directory. This can be done simply by:

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/stevenliu216/568-Final-Project.git
$ cd ~/catkin_ws
```

Move all files not associated with the source code found in the `loam_velodyne` directory to a new location, since you may want to use it later but don't want to have any issues building the project. Next build the project.

```
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
$ source ~/catkin_ws/devel/setup.bash
```

In order to run our code and playback a bag file, in one terminal run:
```
$ roslaunch loam_velodyne loam_velodyne.launch
```

And in a second terminal run:
```
$ rosbag play ~/Downloads/YOUR_BAG_FILE_NAME.bag 
```
On a slower computer, you may want to set the rate setting to a slower rate in order to give your computer more time between playback steps. This can be done by changing .1 to your preferred rate:

```
$ rosbag play -r .1 ~/Downloads/YOUR_BAG_FILE_NAME.bag 
```

You can now play around with the different frames, point cloud objects, etc. to better visualize the LOAM algorithm. We recommend opening a third terminal and typing:
```
$ rqt_graph &
```
to see the flow of data throughout the project. This will also help you debug any issues if your .bag file was formatted incorrectly or if you want to add new features to the code.

## Visualizing Odometry
Install the Rqt Multiplot Plugin tool found [here](https://github.com/ethz-asl/rqt_multiplot_plugin)

On one terminal run:
```
$ roscore
```

On a separate terminal run:
```
$ rqt --force-discover
```
In the menu bar, select plugins -> visualization -> multiplot
Detailed instructions for how to format plots can be found at the github [source](https://github.com/ethz-asl/rqt_multiplot_plugin).

In a third terminal, run this command:
```
$ roslaunch loam_velodyne loam_velodyne.launch
```
And finally, in a fourth terminal, run: 
```
$ rosbag play kitti_bag_file
```
Make sure to hit the play button in top right corner of the plots, after running the kitti .bag file. Allow LOAM to run to completion.

Note: You can also record the topic aft_mapped_to_init or integrated_to_init in a separate bag file, and just use that with rqt_multiplot. This will run much faster.

## Running the Benchmarking Code

In order to run the benchmarking code, which computes errors as well as plots the odometry vs ground truth pose, you will need to echo out the x, y, z positions of the vehicle to a text file which we will then post process.
To do this, open a third terminal and type this command before running the .bag file:
```
$ rostopic echo /laser_odom_to_init/pose/pose/position > FILENAME.txt
```

Next, you will need to download the ground truth data from the KITTI ground truth poses from [here](http://www.cvlibs.net/datasets/kitti/eval_odometry.php). Move your echoed out file and the raw data file to the Benchmarking directory which contains our script. You will need to modify this script to match your filenames but otherwise no additional modification is needed.

## Acknowledgments

Thank you to Maani Ghaffari Jadidi our EECS 568 instructor, as well as the GSIs Lu Gan and Steven Parkison for all the support they provided this semester. You can find a link to our course website [here](http://robots.engin.umich.edu/mobilerobotics/).
We would like to acknowledge Ji Zhang and Sanjiv Singh, for their original papers and source code, as well as Leonid Laboshin for the modified version of Ji Zhang and Sanjiv Singh's code, which was taken down. Leonid's repository can be found [here](https://github.com/laboshinl/loam_velodyne).
