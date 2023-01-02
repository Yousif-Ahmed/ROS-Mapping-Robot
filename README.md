# SUMMIT-XL Mapping Robot

This is a ROS package for controlling the SUMMIT-XL robot in Gazebo and rViz and mapping the environment using the odometry and the laser scanner data.

## Installation

1. Clone the repository

    ```bash
    git clone https://github.com/Yousif-Ahmed/summit-ml-robot.git
    ```

2. Enter the directory and run the setup script

    ```bash
    cd summit-ml-robot
    source setupProject.sh
    ```

## Usage

While in the project directory, run the following commands in separate terminals:

1. Launch Gazebo + rViz simulations

    ```bash
    source runSimulators.sh
    ```

    If you faced an issue such as:

    ```text
    RLException: unused args [arm_manufacturer, arm_model] for include of [/mnt/FC8006E780
    06A7EA/${path to project}/src/summit_xl
    _common/summit_xl_control/launch/summit_xl_control.launch]
    The traceback for the exception was written to the log file
    ```

    Just remove lines 104 and 105 in `src/summit_xl_sim/summit_xl_gazebo/launch/summit_xl_one_robot.launch` file which are:

    ```xml
    <arg name="arm_manufacturer" value="$(arg arm_manufacturer)"/>
    <arg name="arm_model" value="$(arg arm_model)"/>
    ```

2. Launch the IRA Laser Tools

    ```bash
    source runLaserScanner.sh
    ```

3. Launch the mapping

    - To map the environment with known poses

        ```bash
        source runMappingWithKnownPoses.sh
        ```

    - To map the environment using SLAM

        ```bash
        source runSLAM.sh
        ```

    - To run both

        ```bash
        source runMapping.sh
        ```

4. Launch the robot controller

    ```bash
    source runRobotController.sh
    ```

    You can control the robot using the keyboard (WASD)

5. To see the mapping:

    1. In rViz, click on the `Add` button

    2. Select `By topic`

    3. Select the topic `/map_topic` or `/map_slam` depending on the mapping method you used

    4. Give the topic a name (e.g. `Mapping with known poses` or `Mapping with SLAM`)

    5. Click on `Add`

    6. You can hide/show the mapping by toggling the checkbox next to the topic name in the `Displays` tab
