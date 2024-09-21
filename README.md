# SJSU Robotics IntelSysTrialProject

## Cloning the Repository

1. Clone Repository:

    ```bash
    git clone https://github.com/GT-96/SJSU_Robotics_IntelSysTrialProject.git
    ```

2. Navigate into project directory:

    ```bash
    cd SJSU_Robotics_IntelSysTrialProject
    ```

## Building the Package

1. Ensure you are in the root of your workspace (i.e., where the `src` folder is located).

2. Run the following command to build the package:

    ```bash
    colcon build
    ```

3. After the build is complete, source the workspace:

    ```bash
    source install/setup.bash
    ```
## Running the Launch File

Once the package is built and sourced, you can run the launch file, which will start the necessary nodes:

1. Use the following command to run the launch file from the launch directory:

    ```bash
    ros2 launch path_planner path_planner_launch.py
    ```

    This will start all the nodes specified in the launch file, including `goal_and_start`, `map`, `path`, and `viz`.
