## GameOfDronesDev Unity based simulator.
The goal is to make a simulator similar to [Flightmare](https://github.com/uzh-rpg/flightmare) and [FlightGoggles](https://github.com/mit-aera/FlightGoggles) but with custom environments, support for ROS2, and interface with PX4 (for SITL).

Current version provides minimal example of zala drone flying in the sky. In order to add anther environment create a new scene.
Need many improvements. [Also see the docs](docs/docs.md)
Tested on Ubuntu 22.04 and Python 3.10.12. Should work on other platforms, need testing. 


### ROS2
1. Install ROS2 Humble: [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
2. Add sourcing to your shell startup script: [Configuring ROS2 Environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
3. Launch the rosbridge server:
    ```sh
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ```
    If the command does not work, you should install `rosbridge_server`. Google it for more information.



### Unity
1. Install Unity Hub: [Installation Guide](https://docs.unity3d.com/hub/manual/InstallHub.html)
2. In Unity Hub, install Unity editor version 6000.0.24f1
3. Clone the repo 
```
git clone https://github.com/hronoses/GameOfDronesDev.git
```
4. In Unity Hub, add the project from the local folder `/GameOfDronesDev/Unity6`
5. Open the project. Unity editor should start. IN the project panel find DroneDetector1 scene and open it.
6. Press the play button. The game should start building. If successful, it starts streaming information to a ROS2 topic.
6.1 If the unity outputs the error considering pose publisher, find in hierarchy (left panel) ros2 object, open it, in the inspector panel (right) find PoseStamped publisher and on the published transform field select zala (click circle on the right). 
7. In a terminal, write:
    ```sh
    ros2 topic list
    ```
    You should see topics `/camera_image` and `/zala_pose`

## Foxglove
1. Install Foxglove Studio: [Download](https://foxglove.dev/download)
2. Follow the instructions in this [blog post](https://foxglove.dev/blog/using-rosbridge-with-ros2) to set up Foxglove with ROS2.
    ```sh
    sudo apt install ros-$ROS_DISTRO-foxglove-bridge 
    ```
3. Launch the Foxglove bridge:
    ```sh
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml
    ```
4. Open Foxglove Studio and connect to the websocket:
    ```sh
    ws://localhost:8765
    ```
5. Create your own layout, or import the layout `foxglove_layout/GameOfDrones.json` from this repository:
6. If everything is set up correctly, you should see the camera image (blue sky) and the plot of Zala's position in the global frame.

 Summary so far. You have installed ros2 and launched the rosbridge server. It provides the interface with the Unity. Also, you have launched the Unity simulation and output the image and pose topic. To verify that everything works you used a Foxglove studio. Next, the final step is to interface with the python. 

## Python
1. (optional) Create nev virtual environment and activate it
    ```sh
    python -m venv venv
    ```
    - On Windows:
        ```sh
        venv\Scripts\activate
        ```
    - On macOS/Linux:
        ```sh
        source venv/bin/activate
        ```
2. Install requirements
    ```sh
    pip install -r requirements.txt
    ```
3. Launch the camera_view.py 
    ```sh
    python game_of_drones/draft.py
    ```
    
    if everything is ok, you should see the camera image and the gradient of the image using Sobel filters. Note how easier it is to locate the drone on the gradient image.


## TODO
0. Fix the readme and requirements. Make sure other can collaborate
1. Improve rendering and lightening of Zala.
2.  Use physical camera that matches Hq camera for raspberry pi and 6mm wide angle lens.
3.  Add scene with terrain for object tracking research
4. Improve the FPS for large camera resolutions. Consider using WebRTC
5. Add drone detector. 
6. Move the camera from the python script via a new topic.
7. Implement simple direct pursuit targeting law to hit the target. 
8. Add Kalman filter for the target.


<!-- 
## Contributing
 1. Dev chat
 2. Create your own branches for experimenting -->

<!-- ## License -->
<!-- - Specify the license under which the project is distributed. -->

