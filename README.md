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

## Using custom extrinsics for the TF tree
If you use this repository to spawn a fully connected TF-tree for your algorithm and want to use custom extrinsic transforms, a conversion script `multical_to_urdf.py` is provided. You can simply run it by

    rosrun trailblazer_description multical_to_urdf.py <PATH_TO_MULTICAL_RESULTS>

this will collect all extrinsics into a single yaml file and store this in the `config` folder of this package. Note that it also converts the rotations to a Fixed Axis Euler representation as used by the URDF format(more infos on [the official URDF joint documentation](https://wiki.ros.org/urdf/XML/joint#Elements) and on [the ROS rotations documentation](https://wiki.ros.org/action/show/geometry2/RotationMethods)).