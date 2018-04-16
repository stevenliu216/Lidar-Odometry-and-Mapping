# 568-Final-Project

This is Team 18's final project git repository for EECS 568: Mobile Robotics. The title of our project is Visual Lidar Odometry and Mapping with KITTI, and team members include: Abdallah, Alexander Crean, Mohamad Farhat, Alexander Groh, Steven Liu and Christopher Wernette.

You can see the results of the algorithm running here: 

<a href="http://www.youtube.com/watch?feature=player_embedded&v=YTCnmP5RBuE
" target="_blank"><img src="http://img.youtube.com/vi/ZrNLccGeftM/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

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

To install, you will need to clone our loam_velodyne directory into your `~/catkin/src` directory. This can be done simply by:

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
On a slower computer, you may want to set the rate setting to a slower rate in order to give your computer more time between playback steps. This can be done by:

```
$ rosbag play -r .1 ~/Downloads/YOUR_BAG_FILE_NAME.bag 
```

You can now play around with the different frames, point cloud objects, etc. to better visualize the LOAM algorithm. We recommend opening a third terminal and typing:
```
$ rqt_graph &
```
to see the flow of data throughout the project. This will also help you debug any issues if your .bag file was formatted incorrectly or if you want to add new features to the code.

## Running the Benchmarking Code

In order to run the benchmarking code, which computes errors as well as plots the odometry vs ground truth pose, you will need to echo out the x, y, z positions of the vehicle to a text file which we will then post process.
To do this, open a third terminal and type this command before running the .bag file:
```
$ rostopic echo /laser_odom_to_init/pose/pose/position > FILENAME.txt
```

Next, you will need to download the ground truth data from the KITTI ground truth poses from [here](http://www.cvlibs.net/datasets/kitti/eval_odometry.php). Move your echoed out file and the raw data file to the Benchmarking directory which contains our 

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **Billie Thompson** - *Initial work* - [PurpleBooth](https://github.com/PurpleBooth)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone who's code was used
* Inspiration
* etc


# Project Proposal
[Project Proposal](/Project_Proposal.pdf)

# Ji Zhang's Original Source Code
http://docs.ros.org/indigo/api/loam_velodyne/html/files.html
