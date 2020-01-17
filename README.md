# fximu2, ROS Software for FXIMU sensor board

fximu version 2. Utilizes a port of ros's complementary filter on the TM4C123 MCU to generate superior performance.

FXIMU is a sensor board that uses the NXP semiconductor FXOS8700 accelerometer magnetometer and FXAS21002 gyro sensor, and ARM Cortex-M4F Based MCU, TM4C123G as the processor.

### It works with ROS without any problems, does not require drivers, uses standard message types, and is open source.


version 2c, has an external i2c port, which can be used to either connect to an auxiliary sensor, or connect to a RPI in host mode, instead of the USB. also the expansion port is a IDC, and it has 2 LEDs instead of one. Currently the LED is on when the sensor is at steady_state.

this version has 3.2mm dia holes, in order to mount it with plastic screws. JTAG position is corrected, generally it is designed to be more spacious, then the previous one, to accomodate better mounting. The board is as big as a standard matchbox.

![alt text](https://raw.githubusercontent.com/altineller/documentation_images/master/fximu/fximu2c.jpg)

version 2b

![alt text](https://raw.githubusercontent.com/altineller/documentation_images/master/fximu/fximu.jpg)


In the previous version, we were getting raw sensor values to feed ros's imu_complementary_filter on a raspberry PI. With this method we were able to publish at a high frequency, but filtering being off-board caused latency problems.

In this version, we have ported ros's imu_complementary_filter to run on the Texas Instuments TM4C123GH6PM MCU. Sensor data is directly fed to the complementary filter, updated up to 400hz. The filter then queried to get quaternion data at a desired output rate using a output_rate_divider parameter.

Everything is configurable using a fximu_params.yaml file tru rosparam.

```
params/imu: { 
mag_offset_x: 29.76, mag_offset_y: 79.62, mag_offset_z: 73.11,
mag_soft_iron_ix: 0.974, mag_soft_iron_iy: -0.045, mag_soft_iron_iz: -0.004,
mag_soft_iron_jx: -0.045, mag_soft_iron_jy: 0.995, mag_soft_iron_jz: 0.004,
mag_soft_iron_kx: -0.004, mag_soft_iron_ky: 0.004, mag_soft_iron_kz: 1.034,
sensor_read_rate: 400, output_rate_divider: 8,
adaptive_gain: 1, bias_estimation: 1,
gain_acc: 0.02, gain_mag: 0.01, bias_alpha: 0.1,
kAngularVelocityThreshold: 0.06, kAccelerationThreshold: 0.25, kDeltaAngularVelocityThreshold: 0.05,
imu_frame_id: "base_imu_link", mag_frame_id: "mag_imu_link",
gfsr: 3, afsr: 1,
calibration_mode: 0,
steady_limit: 32,
world_frame: 0,
use_mag: 1
}
```

The mag_offset and mag_soft_iron parameters should be filled with data from your calibration software. Sensor board will auto correct for hard and soft iron errors.

`sensor_read_rate` can be 50,100,200, or 400
`output_rate_divider` can be 1,2,4,8,16

The output rate will equal sensor_read_rate / output_rate_divider.

`kAngularVelocityThreshold`, `kAccelerationThreshold` and `kDeltaAngularVelocityThreshold` are thresholds for steady state detection. while the unit is within these thresholds, it will calculate the gyro biases and accel gain.

`adaptive_gain` and `bias_estimation` options can be turned off by setting 0 in the parameters. Notice that in kinetic, true and false are 0 or 1, while in melodic, you can define actual true or false in the yaml file.

`gain_acc`, `gain_mag`, `bias_alpha` are the accel gain, mag gain and bias alpha for the complementary filter, as explained in: 
[http://wiki.ros.org/imu_complementary_filter](http://wiki.ros.org/imu_complementary_filter)

The frame_id's for the respective Imu and MagneticField messages can be setup with the `imu_frame_id` and `mag_frame_id` parameters.

`gfsr` and `afsr` are gyro and accel sensitivities at system init. the defaults are 500DPS for gyro, and 4G for accel.

`calibration_mode` sets the device in calibration mode. it will send raw sensor values as a Int16MultiArray, so the cal_bridge.py script can forward these to a virtual serial port, thus allowing calibration by a 3rd party software without updating the firmware, or removing the sensor from your robot. 

`steady_limit` sets after how many checkStates (returning true), the sensor will be in steady state. This was required to better the adaptive bias feature. Valid values are 2 to 127.

`world_frame` 0 for NWU, 1 for ENU, notice that sensor data is not touched, but output orientation quaternion is rotated.

`use_mag` 0 for not using the mag for fusion. magnetometer will still be read and reported but will not be used for orientation measurements. useful for indoor environments. 1 for using the mag for fusion.

See [CALIBRATION.md](https://github.com/altineller/fximu2/blob/master/CALIBRATION.md) for details.

See [HOWTO.md](https://github.com/altineller/fximu2/blob/master/HOWTO.md) on notes for operations.


### Credits

_Roberto G. Valenti_ for writing the complementary filter.

_Vitor Matos_ for writing the rosserial tivac library.

_Charles Tsai_ and user _cb1_mobile_ from e2e.ti.com forums, for help on debugging the problems on sensor chipset.

_Melih Karakelle_ for advice on the development of the circuit.

### TODO

- Auxillary magnetometer input



