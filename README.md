## bosch_imu_driver
A driver for the sensor IMU Bosch BNO055. It was implemented only the UART communication interface
(correct sensor mode should be selected).

Parameters:
- **port** (default: '/dev/ttyUSB0') - path to USB port where device was connected.
- **frame_id** (default: 'imu_link') - the frame in which sensor data will be published. 
- **frequency** (default: 100) - the frequency of reading from device and publishing data in Hz.
- **operation_mode** (default: OPER_MODE_NDOF) - the operation mode of sensor BNO055. Other modes could be found in sensor [datasheet](https://www.bosch-sensortec.com/bst/products/all_products/bno055).
- **axis_remap_config** (default: 0x24) - the axis remap configuration. Refer to the datasheet page 27.
- **axis_remap_sign** (default: 0x00) - the axis remap sign. Refer to the datasheet page 27.

Publishes:
- **/imu/data** [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
- **/imu/raw** [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
- **/imu/mag** [(sensor_msgs/MagneticField)](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html)
- **/imu/temp** [(sensor_msgs/Temperature)](http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html)
