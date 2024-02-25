
# Autonomous Drone for Infrastructural Inspection

## Project Overview

This repository hosts the codebase for an autonomous drone system designed for infrastructural inspections. The drone leverages advanced algorithms and sensors to navigate and analyze infrastructure, such as bridges, buildings, and power lines, providing valuable data for maintenance and safety assessments.

### Features

- **Autonomous Navigation:** Utilizes GPS and computer vision for precise navigation around buildings.
- **Real-Time Analysis:** Implements machine learning models for immediate identification of structural anomalies.
- **Data Collection:** Gathers high-resolution images and sensor data for further analysis.
- **User-Friendly Interface:** Includes a web-based dashboard for easy monitoring and control of inspection missions.

## Installation

### Prerequisites

- Python 3.8+
- ROS Noetic
- OpenCV 4.5

### Setup

1. Clone the repository:
    ```sh
    git clone https://github.com/bhavya1600/autonomous-drone-for-infrastructural-inspection.git
    ```

2. Install the required Python packages:
    ```sh
    cd autonomous-drone-for-infrastructural-inspection
    pip install -r requirements.txt
    ```

3. Setup ROS environment (follow ROS Noetic installation guide).

4. Build the project:
    ```sh
    catkin_make
    ```

## Usage

To launch the inspection drone, follow these steps:

1. Start the ROS master:
    ```sh
    roscore
    ```

2. Launch the drone interface:
    ```sh
    roslaunch drone_inspection launch_drone.launch
    ```

3. Access the web dashboard at `http://localhost:8080` to monitor the drone's progress and review collected data.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


