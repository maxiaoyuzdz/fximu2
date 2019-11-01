# Notes for Experimenters of the FXIMU Board

#### load parameters 

```rosparam load config/fximu_params.yaml```

Note that roscore must be runnning

#### run rosserial

```rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=230400```


#### run static transform publisher for rviz

```rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map base_imu_link 100```

#### launching test system with a launch file

```roslaunch fximu2 fximu2.launch```

This is a launch file that will launch rosserial, and run the static transform publisher as shown above.

#### run rviz

```rviz```

Note that rviz imu plugin is required to visualize imu data. Click Add on rviz, then select rviz_imu_plugin then type 'imu/data' as topic in the visualization window.

#### echo imu data

```rostopic echo /imu/data```

#### echo mag data

```rostopic echo /imu/mag```

### measure frequency of output data

```rostopic hz /imu/data``` or ```rostopic hz /imu/mag```


### if you are developing on ubuntu

ModemManager service that is default on ubuntu, probes the newly added serial ports, which keeps it busy for few second. To overcome this delay [which is important while developing] turn off the ModemManager service.
