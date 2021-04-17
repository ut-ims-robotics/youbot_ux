## Setting up the on-board computer

Download the desktop image of Ubuntu 18.04.5 from [here](https://releases.ubuntu.com/18.04/).

Install Ubuntu 18.04.5 onto youBots on-board with this [tutorial](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview).

Install ROS with instructions from [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

First, run the following:
```bash
sudo apt install curl
```

Get required packages:
```bash
sudo apt-get install ros-melodic-pr2-msgs ros-melodic-brics-actuator ros-melodic-moveit git ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-control ros-melodic-joy ros-melodic-joystick-drivers ros-melodic-moveit-visual-tools python-rospkg
```

You also need to install these for controller connection:
```bash
sudo apt-get install libbluetooth-dev
sudo apt-get install libcwiid-dev
```

For building the ROS workspace install:
```bash
sudo apt-get install python-catkin-tools
```

Next, you will need a catkin workspace. If you do not have it, use the following commands to create it.

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin build
```

Now source the setup file. So you would not have to source it every time you open a new terminal we will write the source command in the .bashrc file.

$ gedit ~/.bashrc

This will open the text editor. You will need to add this line to the end of the file if it not already there:

source ~/catkin_ws/devel/setup.bash

Save the file and exit.

NB! Before continuing you will have to close and reopen all the terminal windows for the sourcing to work.

Now clone the youbot repositories to our catkin workspace's ~/catkin_ws/src folder:

```bash
git clone -b melodic https://github.com/ScazLab/youbot-manipulation.git
git clone -b hydro-devel https://github.com/youbot/youbot_driver.git
git clone -b kinetic-devel https://github.com/youbot/youbot_description.git
git clone https://github.com/youbot/youbot_driver_ros_interface.git
git clone https://github.com/youbot/youbot_navigation.git
git clone -b noetic-devel https://github.com/ros/executive_smach.git
```

Clone temoto_core and temoto ERM packages according to these [instructions](https://github.com/temoto-telerobotics/temoto_er_manager/tree/feature-standalone).

The following instructions are for controller related packages for ROS:

```bash
$ git clone https://github.com/naoki-mizuno/ds4drv --branch devel
$ cd ds4drv
$ mkdir -p ~/.local/lib/python3.6/site-packages
$ python3 setup.py install --prefix ~/.local
$ sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
$ cd ~/catkin_ws/src
$ git clone https://github.com/naoki-mizuno/ds4_driver.git
```
To attach and connect the DualShock 4 controller to the on-board controller, go to Ubuntu's bluetooth settings, hold the PS button (button in the middle of 2 analog sticks) and the share button until it starts blinking rapidly. After the controller's LED starts blinking, a "Wireless Controller" should show up on the Ubuntu's bluetooth devices menu. Click on it and it should connect.

Now we will get the youbot_ux package for controlling the robot with the controller:
```bash
git clone -b melodic https://github.com/ScazLab/youbot-manipulation.git
```

At this point, build the workspace:
```bash
$ cd ~/catkin_ws/
$ catkin build
```

If you were to run the youBot driver now, it wouldn't find the manipulator nor base motors due to not having the rights and not looking on the right network adapter, so first, lets set the rights (has to be done every time after running "catkin build"):
```bash
$ cd ~
$ sudo setcap cap_net_raw+ep 
~/catkin_ws/devel/.private/youbot_driver_ros_interface/lib/youbot_driver_ros_interface/youbot_driver_ros_interface
$ sudo ldconfig /opt/ros/melodic/lib
```

To select the right network adapter, change the "EthernetDevice =" line in ~/catkin_ws/src/youbot_driver/config/youbot-ethercat.cfg to the adapter being used to connect to the robot. In this case, it should be "EthernetDevice = enx00e04c2151d2" 

Source after building:
```bash
source ~/catkin_ws/devel/setup.bash
```

Next, you should be able to run the youbout_ux package with the following command:
```bash
roslaunch youbot_ux launcher.launch
```

## Launch on system startup with service

Make a file at "/etc/systemd/system/" called "youbot_boot.service" (you may need sudo privileges) and put the following part in the box into it. Make sure to replace the &lt;your username here&gt; parts with the actual username.
```bash
[Unit]
Description=Start Youbot ROS bringup nodes
After=network.target

[Service]
ExecStart=/bin/bash -c "source /home/<your username here>/catkin_ws/devel/setup.bash && roslaunch youbot_ux launcher.launch"
WorkingDirectory=/home/<your username here>
StandardOutput=inherit
StandardError=inherit
Restart=always
User=<your username here>

[Install]
WantedBy=multi-user.target
```

Now run "systemctl daemon-reload"

You can start the service with "systemctl start youbot_boot" and enable the service to start on boot with "systemctl enable youbot_boot"

For future development, "sudo su" and then "systemctl disable youbot_boot" so it would no longer be launched on system boot

## Using the robot
### Startup 
Before starting up the on-board computer, turn on the youBots motors, including the manipulator's motors.

After running the youbot_ux launcher.launch file or after it has automatically been started by a service, turn on your DualShock 4 controller. You can do so by pressing the PS button (button in the middle of 2 analog sticks), after which a blue LED light should start blinking. When it is connected and everything works correctly, the LED should be solid blue.

### Using the robot

When initially starting, the robot will be in "safe mode" with blue light being indicated on the controller. Next, button uses and indicators will be shown in different modes.

#### Safe mode
Safe mode LED indication: Blue  
![Safe Mode Controller](https://raw.githubusercontent.com/ut-ims-robotics/youbot_ux/main/Images/Safe.PNG)
#### Driving mode
Driving mode LED indication: Blue  
![Driving Mode Controller](https://raw.githubusercontent.com/ut-ims-robotics/youbot_ux/main/Images/Driving.PNG)
#### Manipulator control
Manipulator control LED indication: Yellow  
Joint selection vibration indication: joint 2 = 0.1s, joint 3 = 0.3s, joint 4 = 0.5s
![Manipulator Mode Controller](https://raw.githubusercontent.com/ut-ims-robotics/youbot_ux/main/Images/Manipulator.PNG)
#### Teach mode
Teach mode Idle LED indication: Light Blue  
Teach mode Recording LED indication: Red  
Teach mode Playback LED indication: Purple  
![Teach Mode Controller](https://raw.githubusercontent.com/ut-ims-robotics/youbot_ux/main/Images/Teach.PNG)
