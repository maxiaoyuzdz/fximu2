# Calibration Mode

When the `calibration_mode` is set to `true` in the fximu_params.yaml file, the unit outputs raw sensor values at `/imu/array` topic.

The complementary filter is not initialized, and the hard iron and soft iron corrections are not processed, but however `sensor_read_rate`, `output_rate_divider` and parameters `gfsr`, `afsr`on the yaml file are still taken into account.

### Procedure

1. Set `calibration_mode` to true, and select `sensor_read_rate` 100hz and `output_rate_divider` to 2 setting output rate to 50hz, in the fximu_params.yaml file.

2. run `roslaunch fximu2 fximu2.launch` to launch rosserial

3. verify that we are getting raw sensor value arrays by `rostopic echo /imu/array` you should see int values flowing

4. run `socat.sh` provided in scripts directory. This will create `/dev/ttyCAL0` and `/dev/ttyCAL` in which the first one forwards data to the second port.

5. run `perms.sh` provided in the scripts directory. when the `socat.sh` is run, it creates the tty files, which are accessible by root user only, when `perms.sh` is run it will change permissions for the logged in user.

6. run `cal_bridge.py` program provided in the scripts directory. This python program will read data from `/imu/array` topic, and then send those values to `/dev/ttyCAL0`, which will be forwarded to `/dev/ttyCAL1` by socat, as plaintext raw values.

7. verify that the calibration system is running: `screen /dev/ttyCAL 115200` you should see values running as:

> Raw:-73,2054,120,22,-17,51,232,631,277
> Raw:-68,2045,102,51,-56,24,234,617,265
> Raw:-67,2055,122,90,-14,52,245,640,256

To quit screen press `ctrl-A` and `\` keys.

8. Download and compile https://github.com/PaulStoffregen/MotionCal
