## Updating repository manually the last time, so some files will be deleted, but will be put back in 10 minutes

# fximu2, ROS Software for FXIMU sensor board

fximu version 2. Utilizes a port of ros's complementary filter on the TM4C123 MCU to generate superior performance.

FXIMU is a sensor board that uses the NXP semiconductor FXOS8700 accelerometer magnetometer and FXAS21002 gyro sensor, and ARM Cortex-M4F Based MCU, TM4C123G as the processor.

### It works with ROS without any problems, does not require drivers, uses standard message types, and is open source.

![alt text](https://raw.githubusercontent.com/altineller/documentation_images/master/fximu/fximu.jpg)

In the previous version, we were getting raw sensor values to feed ros's imu_complementary_filter on a raspberry PI. With this method we were able to publish at a high frequency, but filtering being off-board caused latency problems.

In this version, we have ported ros's imu_complementary_filter to run on the Texas Instuments TM4C123GH6PM MCU. Sensor data is directly fed to the complementary filter, updated up to 400hz. The filter then queried to get quaternion data at a desired output rate using a output_rate_divider parameter.

Everything is configurable using a fximu_params.yaml file tru rosparam.

```
params/imu: { 
mag_offset_x: 51.00, mag_offset_y: 98.60, mag_offset_z: 31.11, 
mag_soft_iron_ix: 0.990, mag_soft_iron_iy: -0.025, mag_soft_iron_iz: -0.016, 
mag_soft_iron_jx: -0.025, mag_soft_iron_jy: 0.996, mag_soft_iron_jz: 0.011, 
mag_soft_iron_kx: -0.016, mag_soft_iron_ky: 0.011, mag_soft_iron_kz: 1.015, 
sensor_read_rate: 400, output_rate_divider: 4,
adaptive_gain: true, bias_estimation: true,
gain_acc: 0.02, gain_mag: 0.01, bias_alpha: 0.25,
kAngularVelocityThreshold: 0.2, kAccelerationThreshold: 0.4, kDeltaAngularVelocityThreshold: 0.05,
imu_frame_id: "base_imu_link", mag_frame_id: "mag_imu_link",
gfsr: 2, afsr: 1,
calibration_mode: false
}

```

The mag_offset and mag_soft_iron parameters should be filled with data from your calibration software. Sensor board will auto correct for hard and soft iron errors.

`sensor_read_rate` can be 50,100,200, or 400
`output_rate_divider` can be 1,2,4,8,16

The output rate will equal sensor_read_rate / output_rate_divider.

`kAngularVelocityThreshold`, `kAccelerationThreshold` and `kDeltaAngularVelocityThreshold` are thresholds for steady state detection. while the unit is within these thresholds, it will calculate the gyro biases and accel gain.

`adaptive_gain` and `bias_estimation` options can be turned off by setting false in the parameters.

`gain_acc`, `gain_mag`, `bias_alpha` are the accel gain, mag gain and bias alpha for the complementary filter, as explained in: 
[http://wiki.ros.org/imu_complementary_filter](http://wiki.ros.org/imu_complementary_filter)

The frame_id's for the respective Imu and MagneticField messages can be setup with the `imu_frame_id` and `mag_frame_id` parameters.

`gfsr` and `afsr` are gyro and accel sensitivities at system init. the defaults are 500DPS for gyro, and 4G for accel.

`calibration_mode` sets the device in calibration mode. it will send raw sensor values as a Int16MultiArray, so the cal_bridge.py script can forward these to a virtual serial port, thus allowing calibration by a 3rd party software without updating the firmware, or removing the sensor from your robot. 

See [CALIBRATION.md](https://github.com/altineller/fximu2/blob/master/CALIBRATION.md) for details.

See [HOWTO.md](https://github.com/altineller/fximu2/blob/master/HOWTO.md) on notes for operations.


### Credits

_Roberto G. Valenti_ for writing the complementary filter.

_Vitor Matos_ for writing the rosserial tivac library.

_Charles Tsai_ and user _cb1_mobile_ from e2e.ti.com forums, for help on debugging the problems on sensor chipset.

_Melih Karakelle_ for advice on the development of the circuit.

