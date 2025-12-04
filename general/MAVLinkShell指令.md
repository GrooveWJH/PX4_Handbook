# MAVLink Shell（NSH）可用命令清单

本清单根据仓库内 `docs/en/modules/*.md` 自动提取，列出在 PX4 固件中实现了命令行接口的模块/命令名称。它们可在 MAVLink Shell（或系统控制台）中使用，前提是对应模块已编译进固件。实际可用集合以运行时 `help` 输出为准。

## 获取实时列表
- 在 MAVLink Shell 或系统控制台输入 `help` 查看当前固件内全部命令。
- 大多数命令支持 `<command> help` 输出用法。

## 命令分组列表
按模块文档分类整理（括号为每组命令数量）。

### autotune (2)
- fw_autotune_attitude_control
- mc_autotune_attitude_control

### command (27)
- actuator_test
- bl_update
- bsondump
- dumpfile
- dyn
- failure
- gpio
- hardfault_log
- hist
- i2cdetect
- led_control
- listener
- mfd
- mft_cfg
- mtd
- nshterm
- param
- payload_deliverer
- perf
- reboot
- sd_bench
- sd_stress
- serial_passthru
- system_time
- top
- usb_connected
- ver

### communication (3)
- frsky_telemetry
- mavlink
- uorb

### controller (19)
- airship_att_control
- control_allocator
- flight_mode_manager
- fw_att_control
- fw_lat_lon_control
- fw_mode_manager
- fw_rate_control
- mc_att_control
- mc_nn_control
- mc_pos_control
- mc_rate_control
- navigator
- rover_ackermann
- rover_differential
- rover_mecanum
- spacecraft
- uuv_att_control
- uuv_pos_control
- vtol_att_control

### driver (51)
- MCP23009
- atxxxx
- batmon
- batt_smbus
- bst
- dshot
- fake_gps
- fake_imu
- fake_magnetometer
- ft_technologies_serial
- gimbal
- gps
- gz_bridge
- ina220
- ina226
- ina228
- ina238
- iridiumsbd
- irlock
- linux_pwm_out
- lsm303agr
- mcp9808
- msp_osd
- newpixel
- paa3905
- paw3902
- pca9685_pwm_out
- pm_selector_auterion
- pmw3901
- pps_capture
- pwm_out
- pwm_out_sim
- px4flow
- px4io
- rgbled
- rgbled_aw2023
- rgbled_is31fl3195
- rgbled_lp5562
- roboclaw
- rpm_capture
- safety_button
- septentrio
- sht3x
- tap_esc
- tone_alarm
- uwb
- vertiq_io
- voxl2_io
- voxl_esc
- voxlpm
- zenoh

### driver_adc (4)
- TLA2528
- adc
- ads1115
- ads7953

### driver_airspeed_sensor (7)
- asp5033
- auav
- ets_airspeed
- ms4515
- ms4525do
- ms5525dso
- sdp3x

### driver_baro (15)
- bmp280
- bmp388
- bmp581
- dps310
- icp101xx
- icp201xx
- lps22hb
- lps25h
- lps33hw
- mpc2520
- mpl3115a2
- ms5611
- ms5837
- spa06
- spl06

### driver_camera (1)
- camera_trigger

### driver_distance_sensor (19)
- afbrs50
- gy_us42
- leddar_one
- lightware_laser_i2c
- lightware_laser_serial
- lightware_sf45_serial
- ll40ls
- ll40ls_pwm
- mappydot
- mb12xx
- pga460
- srf02
- srf05
- teraranger
- tf02pro
- tfmini
- ulanding_radar
- vl53l0x
- vl53l1x

### driver_imu (34)
- adis16448
- adis16470
- adis16477
- adis16497
- adis16507
- bmi055
- bmi085
- bmi088
- bmi088_i2c
- bmi270
- fxas21002c
- fxos8701cq
- iam20680hp
- icm20602
- icm20608g
- icm20649
- icm20689
- icm20948
- icm20948_i2c_passthrough
- icm40609d
- icm42605
- icm42670p
- icm42688p
- icm45686
- iim42652
- iim42653
- l3gd20
- lsm303d
- lsm9ds1
- mpu6000
- mpu9250
- mpu9250_i2c
- mpu9520
- sch16t

### driver_ins (5)
- MicroStrain
- eulernav_bahrs
- ilabs
- sbgecom
- vectornav

### driver_magnetometer (15)
- ak09916
- ak8963
- bmm150
- bmm350
- hmc5883
- iis2mdc
- ist8308
- ist8310
- lis3mdl
- lsm9ds1_mag
- mmc5983ma
- qmc5883l
- qmc5883p
- rm3100
- vcm1193l

### driver_optical_flow (1)
- thoneflow

### driver_radio_control (5)
- crsf_rc
- dsm_rc
- ghst_rc
- rc_input
- sbus_rc

### driver_rpm_sensor (1)
- pcf8583

### driver_transponder (1)
- sagetech_mxs

### estimator (5)
- AttitudeEstimatorQ
- airspeed_estimator
- ekf2
- local_position_estimator
- mc_hover_thrust_estimator

### simulation (1)
- simulator_sih

### system (37)
- battery_simulator
- battery_status
- camera_feedback
- cdcacm_autostart
- commander
- dataman
- dmesg
- esc_battery
- gyro_calibration
- gyro_fft
- hardfault_stream
- heater
- i2c_launcher
- internal_combustion_engine_control
- land_detector
- load_mon
- logger
- mag_bias_estimator
- manual_control
- netman
- pwm_input
- rc_update
- replay
- send_event
- sensor_agp_sim
- sensor_arispeed_sim
- sensor_baro_sim
- sensor_gps_sim
- sensor_mag_sim
- sensors
- system_power_simulation
- tattu_can
- temperature_compensation
- time_persistor
- tune_control
- uxrce_dds_client
- work_queue

### template (2)
- module
- work_item_example
