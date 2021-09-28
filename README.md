# Using Franka Emika Panda Robot in a Webots Environment
## Creating Franka Proto File
- Obtain the panda_description folder located in the git repository ros-planning/moveit_resources (https://github.com/ros-planning/moveit_resources/tree/master/panda_description)
- Look at the urdf/panda.urdf file and change any filepath to your local machine filepath
- Download the source code for cyberbotics/urdf2webots (https://github.com/cyberbotics/urdf2webots)
- Follow the instructions in the urdf2webots repository to convert the urdf to proto robot description 
## Using the Franka Proto File
- Add the proto robot path to your webots world
- Load the proper DH and position/torque parameters from the Franka Emika FCI documentation website (https://frankaemika.github.io/docs/control_parameters.html)
- Address any errors displayed on the webots terminal when loading the proto file