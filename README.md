# trailblazer_description
This repository contains a ROS package with URDF files for Trailblazer.

![image](https://user-images.githubusercontent.com/32697515/223465138-8c24067e-c567-4896-87aa-74ea7f63fcfb.png)


## Usage
Clone the repository into your catkin workspace and build it

    cd src
    git clone https://github.com/Hilti-Research/trailblazer_description.git
    catkin build trailblazer_description
    source ../devel/setup.bash
    
Then, launch the visualization with

    roslaunch trailblazer_description visualize.launch
    
or use the description within your project by including the `load.launch` file into your launch file (don't forget to also spawn a `robot_state_publisher`)

    <include file="$(find trailblazer_description)/launch/load.launch" />
