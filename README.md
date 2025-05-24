# Virya Coding Assignment

The instructions for this assignment can be found in the [assignment_instructions.pdf](./assignment_instructions.pdf) file included in this repository.  
This README provides only the installation instructions.

## Requirements
- **Operating System**: Ubuntu 22.04
- **ROS 2 Distribution**: Humble
- **Ignition**: Pre-installed on the system

If the above requirements are not met, you can use the provided Docker setup.

## Installation Instructions

### Native Installation
1. Ensure you are running Ubuntu 22.04. If not, refer to the [Docker Installation](#docker-installation) section.
2. Install ROS 2 Humble by following the [official ROS 2 installation guide](https://docs.ros.org/en/).
3. Verify that Ignition is installed and properly configured.
4. Clone this repository:
    ```bash
    git clone https://github.com/virya-ai/virya_coding_assignment.git
    cd virya_coding_assignment
    ```
5. Initialize `rosdep` (if not already initialized):
    ```bash
    sudo rosdep init
    rosdep update
    ```
6. Install dependencies using `rosdep`:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
7. Build the workspace:
    ```bash
    colcon build
    ```
8. Source the workspace:
    ```bash
    source install/setup.bash
    ```

### Docker Installation
1. If you do not have Ubuntu 22.04 on your system, you can run the assignment using Docker. Ensure Docker is installed on your system. If not, follow the [Docker installation guide](https://docs.docker.com/get-docker/).  
    You must also be able to run Docker without `sudo`. If you cannot, follow the steps in the [Linux post-installation guide](https://docs.docker.com/engine/install/linux-postinstall/).
2. Clone this repository (if you haven't already):
     ```bash
     git clone https://github.com/virya-ai/virya_coding_assignment.git
     cd virya_coding_assignment
     ```
3. Build the Docker image (note: pulling the base image and building may take several minutes):
     ```bash
     docker build -t virya_ros2_humble .
     ```
4. Run the Docker container:
     ```bash
     bash docker_run.bash
     ```

## Assignment Instructions
Refer to the `assignment_instructions.pdf` file in the repository for detailed instructions on completing the assignment.

## Running the Assignment

### Native ROS 2 Run
1. Ensure the workspace is built and sourced:
    ```bash
    colcon build
    source install/setup.bash
    ```

2. Launch the assignment:
    ```bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo virya_test.launch.py
    ```

### Docker Run
1. Use the provided script to run and access the Docker container:
    ```bash
    ./docker_run.bash
    ```

2. Once inside the container, launch the assignment:
    ```bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo virya_test.launch.py
    ```

3. To open additional terminals inside the running container, use the following command from your host system:
    ```bash
    docker exec -it virya_test_container bash
    ```
