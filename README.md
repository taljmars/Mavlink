ArduCopter Parameters

Name | Possible Value | Increment | Unit | Range | Read Only | Title | Description
--- | --- | --- | --- | --- | ---| --- | ---
ACRO_BAL_PITCH | 1 | 0.1 | unknown | 0 3 | false | Acro Balance Pitch | rate at which pitch angle returns to level in acro mode. A higher value causes the vehicle to return to level faster.
ACRO_BAL_ROLL | 1 | 0.1 | unknown | 0 3 | false | Acro Balance Roll | rate at which roll angle returns to level in acro mode. A higher value causes the vehicle to return to level faster.
ACRO_EXPO | 0.3 | 1 | unknown | | false |  | Coming soon
ACRO_RP_P | 4.5 | 1 |  | | false |  | Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode. Higher values mean faster rate of rotation.
ACRO_TRAINER | 2 | 1 | list | 0:Disabled 1:Leveling 2:LevelingAndLimited  | false | Acro Trainer | 
ACRO_YAW_P | 4.5 | 1 |  | | false |  | Converts pilot yaw input into a desired rate of rotation in ACRO, Stabilize and SPORT modes. Higher values mean faster rate of rotation.
AHRS_COMP_BETA | 0.1 | 1 | unknown | 0.001 0.5 | false |  | This controls the time constant for the cross-over frequency used to fuse AHRS (MAV_PARAM_GROUP.ARDUCOPTER,airspeed and heading) and GPS data to estimate ground velocity. Time constant is 0.1/beta. A larger time constant will use GPS data less and a small time constant will use air data less.
AHRS_GPS_GAIN | 1 | 1 | unknown | 0.0 1.0 | false |  | This controls how how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.
AHRS_GPS_MINSATS | 6 | 1 | unknown | 0 10 | false |  | Minimum number of satellites visible to use GPS for velocity based corrections attitude correction. This defaults to 6, which is about the point at which the velocity numbers from a GPS become too unreliable for accurate correction of the accelerometers.
AHRS_GPS_USE | 1 | 1 | list | 0:Disabled 1:Enabled  | false |  | 
AHRS_ORIENTATION | 0 | 1 | list | 0:None 1:Yaw45 2:Yaw90 3:Yaw135 4:Yaw180 5:Yaw225 6:Yaw270 7:Yaw315 8:Roll180 9:Roll180Yaw45 10:Roll180Yaw90 11:Roll180Yaw135 12:Pitch180 13:Roll180Yaw225 14:Roll180Yaw270 15:Roll180Yaw315 16:Roll90 17:Roll90Yaw45 18:Roll90Yaw90 19:Roll90Yaw135 20:Roll270 21:Roll270Yaw45 22:Roll270Yaw90 23:Roll270Yaw136 24:Pitch90 25:Pitch270 26:Pitch180Yaw90 27:Pitch180Yaw270 28:Roll90Pitch90 29:Roll180Pitch90 30:Roll270Pitch90 31:Roll90Pitch180 32:Roll270Pitch180 33:Roll90Pitch270 34:Roll180Pitch270 35:Roll270Pitch270 36:Roll90Pitch180Yaw90 37:Roll90Yaw270  | false |  | 
AHRS_RP_P | 0.1 | 1 | unknown | 0.1 0.4 | false |  | This controls how fast the accelerometers correct the attitude
AHRS_TRIM_X | 4.882812E-4 | 1 | radian | -0.1745 0.1745 | false |  | Compensates for the roll angle difference between the control board and the frame. Positive values make the vehicle roll right.
AHRS_TRIM_Y | 0.02518386 | 1 | radian | -0.1745 0.1745 | false |  | Compensates for the pitch angle difference between the control board and the frame. Positive values make the vehicle pitch up/back.
AHRS_TRIM_Z | 0 | 1 | radian | -0.1745 0.1745 | false |  | Not Used
AHRS_WIND_MAX | 0 | 1 | m/s | 0 127 | false |  | This sets the maximum allowable difference between ground speed and airspeed. This allows the plane to cope with a failing airspeed sensor. A value of zero means to use the airspeed as is.
AHRS_YAW_P | 0.1 | 1 | unknown | 0.1 0.4 | false |  | This controls the weight the compass or GPS has on the heading. A higher value means the heading will track the yaw source (MAV_PARAM_GROUP.ARDUCOPTER,GPS or compass) more rapidly.
ANGLE_MAX | 4500 | 1 | centidegree | 1000 8000 | false |  | Maximum lean angle in all flight modes
ARMING_CHECK | 94 | 1 | list | 0:Disabled -17:SkipINS -33:SkipParams/Rangefinder -65:SkipRC 1:Enabled -3:SkipBaro -5:SkipCompass -9:SkipGPS 127:SkipVoltage  | false |  | Allows enabling or disabling of pre-arming checks of receiver, accelerometer, barometer, compass and GPS
ATC_ACCEL_RP_MAX | 0 | 1 | unknown | | false |  | Coming soon
ATC_ACCEL_Y_MAX | 0 | 1 | list | 18000:Slow 36000:Medium 54000:Fast 720000:Disabled  | false |  | Maximum acceleration in yaw axis
ATC_RATE_FF_ENAB | 0 | 1 | list | 0:Disabled 1:Enabled  | false |  | Controls whether body-frame rate feedforward is enabled or disabled
ATC_RATE_RP_MAX | 18000 | 1 | unknown | | false |  | Coming soon
ATC_RATE_Y_MAX | 9000 | 1 | unknown | | false |  | Coming soon
ATC_SLEW_YAW | 1000 | 1 | centidegrees/s | 500 18000 | false |  | Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
BAROGLTCH_ACCEL | 1500 | 1 | unknown | | false |  | Coming soon
BAROGLTCH_DIST | 500 | 1 | unknown | | false |  | Coming soon
BAROGLTCH_ENABLE | 1 | 1 | unknown | | false |  | Coming soon
BATT_AMP_OFFSET | 0 | 1 | volt | | false |  | Voltage offset at zero current on current sensor
BATT_CAPACITY | 4000 | 1 | mAh | | false |  | Capacity of the battery in MILLIAMPER_PER_HOUR when full
BATT_CURR_PIN | 3 | 1 | list | -1:Disabled 1:A1 2:A2 3:Pixhawk 101:PX4 12:A12  | false |  | Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk powered from the PM connector it should be set to 3.
BATT_MONITOR | 4 | 1 | list | 0:Disabled 3:Analog_Voltage_Only 4:Analog_Voltage_and_Current 5:SMBus 6:Bebop  | false |  | Controls enabling monitoring of the battery's voltage and current
BATT_VOLT_MULT | 10.1 | 1 | unknown | | false |  | Used to convert the voltage of the voltage sensing pin (MAV_PARAM_GROUP.ARDUCOPTER,BATT_VOLT_PIN) to the actual battery's voltage (MAV_PARAM_GROUP.ARDUCOPTER,pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX4 using the PX4IO power supply this should be set to 1.
BATT_VOLT_PIN | 2 | 1 | list | -1:Disabled 0:A0 1:A1 2:Pixhawk 100:PX4 13:A13  | false |  | Setting this to 0 ~ 13 will enable battery voltage sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
BATT_VOLT2_MULT | 1 | 1 | unknown | | false |  | Coming soon
BATT_VOLT2_PIN | -1 | 1 | unknown | | false |  | Coming soon
CAM_DURATION | 10 | 1 | second | 0 50 | false |  | How long the shutter will be held open in 10ths of a second (MAV_PARAM_GROUP.ARDUCOPTER,i.e. enter 10 for 1second, 50 for 5seconds)
CAM_SERVO_OFF | 1100 | 1 | PWM | 1000 2000 | false |  | PWM value to move servo to when shutter is deactivated
CAM_SERVO_ON | 1300 | 1 | PWM | 1000 2000 | false |  | PWM value to move servo to when shutter is activated
CAM_TRIGG_DIST | 0 | 1 | meter | 0 1000 | false |  | Distance in meters between camera triggers. If this value is non-zero then the camera will trigger whenever the GPS position changes by this number of meters regardless of what mode the APM is in. Note that this parameter can also be set in an auto mission using the DO_SET_CAM_TRIGG_DIST command, allowing you to enable/disable the triggering of the camera during the flight.
CAM_TRIGG_TYPE | 0 | 1 | list | 0:Servo 1:Relay  | false |  | how to trigger the camera to take a picture
CH7_OPT | 17 | 1 | list | 0:DoNothing 2:Flip 3:SimpleMode 4:RTL 5:SaveTrim 7:SaveWP 9:CameraTrigger 10:RangeFinder 11:Fence 13:SuperSimpleMode 14:AcroTrainer 15:Sprayer 16:Auto 17:AutoTune 18:Land 19:EPM 21:ParachuteEnable 22:ParachuteRelease 23:Parachute3pos 24:AutoMissionReset 25:AttConFeedForward 26:AttConAccelLimits 27:RetractMount 28:Relay-On/Off 29:Landing-Gear 30:Lost-Copter-Sound 31:Motor-Emergency-Stop 32:MotorInterlock 33:Brake 34:Relay2-On/Off 35:Relay3-On/Off 36:Relay4-On/Off 37:Throw 38:Avoidance  | false |  | Select which function is performed when CH7 is above 1800 pwm
CH8_OPT | 0 | 1 | list | 0:DoNothing 2:Flip 3:SimpleMode 4:RTL 5:SaveTrim 7:SaveWP 9:CameraTrigger 10:RangeFinder 11:Fence 13:SuperSimpleMode 14:AcroTrainer 15:Sprayer 16:Auto 17:AutoTune 18:Land 19:EPM 21:ParachuteEnable 22:ParachuteRelease 23:Parachute3pos 24:AutoMissionReset 25:AttConFeedForward 26:AttConAccelLimitsRetractMount 28:Relay-On/Off 29:Landing-Gear 30:Lost-Copter-Sound 31:Motor-Emergency-Stop 32:MotorInterlock 33:Brake 34:Relay2-On/Off 35:Relay3-On/Off 36:Relay4-On/Off 37:Throw 38:Avoidance  | false |  | Select which function is performed when CH8 is above 1800 pwm
CIRCLE_RADIUS | 500 | 1 | cm | 0 10000 | false |  | Defines the radius of the circle the vehicle will fly when in Circle flight mode
CIRCLE_RATE | 20 | 1 | deg/s | -90 90 | false |  | Circle mode's turn rate in deg/sec. Positive to turn clockwise, negative for counter clockwise
COMPASS_AUTODEC | 1 | 1 | list | 0:Disabled 1:Enabled  | false |  | Enable or disable the automatic calculation of the declination based on gps location
COMPASS_DEC | 0.06941485 | 1 | radian | -3.142 3.142 | false |  | An angle to compensate between the true north and magnetic north
COMPASS_EXTERNAL | 0 | 1 | list | 0:Internal 1:External 2:ForcedExternal  | false |  | Configure compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
COMPASS_LEARN | 0 | 1 | list | 0:Disabled 1:Internal-Learning 2:EKF-Learning  | false |  | Enable or disable the automatic learning of compass offsets. You can enable learning either using a compass-only method that is suitable only for fixed wing aircraft or using the offsets learnt by the active EKF state estimator. If this option is enabled then the learnt offsets are saved when you disarm the vehicle.
COMPASS_MOT_X | 0 | 1 | milligauss/amper | -1000 1000 | false |  | Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference
COMPASS_MOT_Y | 0 | 1 | milligauss/amper | -1000 1000 | false |  | Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference
COMPASS_MOT_Z | 0 | 1 | milligauss/amper | -1000 1000 | false |  | Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference
COMPASS_MOTCT | 0 | 1 | list | 0:Disabled 1:UseThrottle 2:UseCurrent  | false |  | Set motor interference compensation type to disabled, throttle or current. Do not change manually.
COMPASS_OFS_X | -98 | 1 | milligauss | -400 400 | false |  | Offset to be added to the compass x-axis values to compensate for metal in the frame
COMPASS_OFS_Y | 29 | 1 | milligauss | -400 400 | false |  | Offset to be added to the compass y-axis values to compensate for metal in the frame
COMPASS_OFS_Z | -18 | 1 | milligauss | -400 400 | false |  | Offset to be added to the compass z-axis values to compensate for metal in the frame
COMPASS_ORIENT | 0 | 1 | list | 0:None 1:Yaw45 2:Yaw90 3:Yaw135 4:Yaw180 5:Yaw225 6:Yaw270 7:Yaw315 8:Roll180 9:Roll180Yaw45 10:Roll180Yaw90 11:Roll180Yaw135 12:Pitch180 13:Roll180Yaw225 14:Roll180Yaw270 15:Roll180Yaw315 16:Roll90 17:Roll90Yaw45 18:Roll90Yaw90 19:Roll90Yaw135 20:Roll270 21:Roll270Yaw45 22:Roll270Yaw90 23:Roll270Yaw136 24:Pitch90 25:Pitch270 26:Pitch180Yaw90 27:Pitch180Yaw270 28:Roll90Pitch90 29:Roll180Pitch90 30:Roll270Pitch90 31:Roll90Pitch180 32:Roll270Pitch180 33:Roll90Pitch270 34:Roll180Pitch270 35:Roll270Pitch270 36:Roll90Pitch180Yaw90 37:Roll90Yaw270 38:Yaw293Pitch68Roll90  | false |  | The orientation of the compass relative to the autopilot board. This will default to the right value for each board type, but can be changed if you have an external compass. See the documentation for your external compass for the right value. The correct orientation should give the X axis forward, the Y axis to the right and the Z axis down. So if your aircraft is pointing west it should show a positive value for the Y axis, and a value close to zero for the X axis. On a PX4 or Pixhawk with an external compass the correct value is zero if the compass is correctly oriented. NOTE: This orientation is combined with any AHRS_ORIENTATION setting.
COMPASS_USE | 1 | 1 | list | 0:Disabled 1:Enabled  | false |  | Enable or disable the use of the compass (MAV_PARAM_GROUP.ARDUCOPTER,instead of the GPS) for determining heading
DCM_CHECK_THRESH | 0.8 | 1 | unknown | | false |  | Coming soon
EKF_CHECK_THRESH | 0.8 | 1 | unknown | | false |  | Coming soon
ESC | 0 | 1 | unknown | | false |  | Coming soon
FENCE_ACTION | 1 | 1 | list | 0:ReportOnly 1:RTL-or-Land  | false |  | What action should be taken when fence is breached
FENCE_ALT_MAX | 100 | 1 | meter | 10 1000 | false | Fence Maximum Altitude | Maximum altitude allowed before geofence triggers
FENCE_ENABLE | 1 | 1 | list | 0:Disabled 1:Enabled  | false |  | Allows you to enable (MAV_PARAM_GROUP.ARDUCOPTER,1) or disable (MAV_PARAM_GROUP.ARDUCOPTER,0) the fence functionality
FENCE_MARGIN | 2 | 1 | meter | 1 10 | false | Fence Margin | Distance that autopilot's should maintain from the fence to avoid a breach
FENCE_RADIUS | 300 | 1 | meter | 30 10000 | false |  | Circle fence radius which when breached will cause an RTL
FENCE_TYPE | 1 | 1 | list | 0:None 1:Altitude 2:Circle 3:AltitudeAndCircle 4:Polygon 5:AltitudeAndPolygon 6:CircleAndPolygon 7:All  | false |  | Enabled fence types held as bitmask
FLOW_ENABLE | 0 | 1 | list | 0:Disabled 1:Enabled  | false |  | Setting this to Enabled(MAV_PARAM_GROUP.ARDUCOPTER,1) will enable optical flow. Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will disable optical flow
FLTMODE1 | 0 | 1 | list | 0:Stabilize 1:Acro 2:AltHold 3:Auto 4:Guided 5:Loiter 6:RTL 7:Circle 9:Land 11:Drift 13:Sport 14:Flip 15:AutoTune 16:PosHold 17:Brake 18:Throw 19:Avoid_ADSB 20:Guided_NoGPS  | false |  | Flight mode when Channel 5 pwm is <= 1230
FLTMODE2 | 0 | 1 | list | 0:Stabilize 1:Acro 2:AltHold 3:Auto 4:Guided 5:Loiter 6:RTL 7:Circle 9:Land 11:Drift 13:Sport 14:Flip 15:AutoTune 16:PosHold 17:Brake 18:Throw 19:Avoid_ADSB 20:Guided_NoGPS  | false |  | Flight mode when Channel 5 pwm is >1230, <= 1360
FLTMODE3 | 16 | 1 | list | 0:Stabilize 1:Acro 2:AltHold 3:Auto 4:Guided 5:Loiter 6:RTL 7:Circle 9:Land 11:Drift 13:Sport 14:Flip 15:AutoTune 16:PosHold 17:Brake 18:Throw 19:Avoid_ADSB 20:Guided_NoGPS  | false |  | Flight mode when Channel 5 pwm is >1360, <= 1490
FLTMODE4 | 16 | 1 | list | 0:Stabilize 1:Acro 2:AltHold 3:Auto 4:Guided 5:Loiter 6:RTL 7:Circle 9:Land 11:Drift 13:Sport 14:Flip 15:AutoTune 16:PosHold 17:Brake 18:Throw 19:Avoid_ADSB 20:Guided_NoGPS  | false |  | Flight mode when Channel 5 pwm is >1490, <= 1620
FLTMODE5 | 6 | 1 | list | 0:Stabilize 1:Acro 2:AltHold 3:Auto 4:Guided 5:Loiter 6:RTL 7:Circle 9:Land 11:Drift 13:Sport 14:Flip 15:AutoTune 16:PosHold 17:Brake 18:Throw 19:Avoid_ADSB 20:Guided_NoGPS  | false |  | Flight mode when Channel 5 pwm is >1620, <= 1749
FLTMODE6 | 6 | 1 | list | 0:Stabilize 1:Acro 2:AltHold 3:Auto 4:Guided 5:Loiter 6:RTL 7:Circle 9:Land 11:Drift 13:Sport 14:Flip 15:AutoTune 16:PosHold 17:Brake 18:Throw 19:Avoid_ADSB 20:Guided_NoGPS  | false |  | Flight mode when Channel 5 pwm is >=1750
FRAME | 1 | 1 | list | 0:Plus 1:X 2:V 3:H 4:V-Tail 5:A-Tail 10:Y6B-New)  | false |  | Controls motor mixing for multicopters. Not used for Tri or Traditional Helicopters.
FS_BATT_ENABLE | 0 | 1 | list | 0:Disabled 1:Land 2:RTL  | false |  | Controls whether failsafe will be invoked when battery voltage or current runs low
FS_BATT_MAH | 0 | 1 | mAh | | false |  | Battery capacity remaining to trigger failsafe. Set to 0 to disable battery remaining failsafe. If the battery remaining drops below this level then the copter will RTL
FS_BATT_VOLTAGE | 14 | 1 | volt | | false |  | Battery voltage to trigger failsafe. Set to 0 to disable battery voltage failsafe. If the battery voltage drops below this voltage then the copter will RTL
FS_GCS_ENABLE | 0 | 1 | list | 0:Disabled 1:Enabled_always_RTL 2:Enabled_Continue_with_Mission_in_Auto_Mode  | false |  | Controls whether failsafe will be invoked (MAV_PARAM_GROUP.ARDUCOPTER,and what action to take) when connection with Ground station is lost for at least 5 seconds. NB. The GCS Failsafe is only active when RC_OVERRIDE is being used to control the vehicle.
FS_GPS_ENABLE | 1 | 1 | unknown | | false |  | Coming soon
FS_THR_ENABLE | 1 | 1 | list | 0:Disabled 1:Enabled_Always_RTL 2:Enabled_Continue_with_Mission_in_Auto_Mode 3:Enabled_Always_LAND  | false |  | The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
FS_THR_VALUE | 975 | 1 | PWM | 925 1100 | false |  | The PWM level on channel 3 below which throttle failsafe triggers
GND_ABS_PRESS | 50362.25 | 1 | pascal | | true |  | calibrated ground pressure in PASCAL
GND_ALT_OFFSET | 0 | 1 | meter | | false |  | altitude offset in meters added to barometric altitude. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer. The value is added to the barometric altitude read by the aircraft. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed.
GND_TEMP | 29.32966 | 1 | degrees-celsius | | false |  | calibrated ground temperature in degrees Celsius
GPS_HDOP_GOOD | 230 | 1 | unknown | 100 900 | false |  | GPS Hdop value at or below this value represent a good position. Used for pre-arm checks
GPS_NAVFILTER | 8 | 1 | list | 0:Portable 2:Stationary 3:Pedestrian 4:Automotive 5:Sea 6:Airborne1G 7:Airborne2G 8:Airborne4G  | false |  | Navigation filter engine setting
GPS_TYPE | 1 | 1 | list | 0:None 1:AUTO 2:uBlox 3:MTK 4:MTK19 5:NMEA 6:SiRF 7:HIL 8:SwiftNav 9:PX4-UAVCAN 10:SBF 11:GSOF 12:QURT 13:ERB 14:MAV 15:NOVA  | false |  | GPS type
GPSGLITCH_ACCEL | 1000 | 1 | unknown | | false |  | Coming soon
GPSGLITCH_ENABLE | 1 | 1 | unknown | | false |  | Coming soon
GPSGLITCH_RADIUS | 200 | 1 | unknown | | false |  | Coming soon
HLD_LAT_P | 1 | 1 | unknown | | false |  | Coming soon
INAV_TC_XY | 2.5 | 1 | unknown | | false |  | Coming soon
INAV_TC_Z | 5 | 1 | unknown | | false |  | Coming soon
INS_ACCOFFS_X | 0.02704384 | 1 | m/s/s | -3.5 3.5 | false |  | Accelerometer offsets of X axis. This is setup using the acceleration calibration or level operations
INS_ACCOFFS_Y | 0.07232188 | 1 | m/s/s | -3.5 3.5 | false |  | Accelerometer offsets of Y axis. This is setup using the acceleration calibration or level operations
INS_ACCOFFS_Z | -0.2916607 | 1 | m/s/s | -3.5 3.5 | false |  | Accelerometer offsets of Z axis. This is setup using the acceleration calibration or level operations
INS_ACCSCAL_X | 0.9998162 | 1 | unknown | 0.8 1.2 | false |  | Accelerometer scaling of X axis. Calculated during acceleration calibration routine
INS_ACCSCAL_Y | 0.9920868 | 1 | unknown | 0.8 1.2 | false |  | Accelerometer scaling of Y axis Calculated during acceleration calibration routine
INS_ACCSCAL_Z | 1.0001 | 1 | unknown | 0.8 1.2 | false |  | Accelerometer scaling of Z axis Calculated during acceleration calibration routine
INS_GYROFFS_X | -0.003293759 | 1 | rad/s | | false |  | Gyro sensor offsets of X axis. This is setup on each boot during gyro calibrations
INS_GYROFFS_Y | -0.07851811 | 1 | rad/s | | false |  | Gyro sensor offsets of Y axis. This is setup on each boot during gyro calibrations
INS_GYROFFS_Z | 0.02421099 | 1 | rad/s | | false |  | Gyro sensor offsets of Z axis. This is setup on each boot during gyro calibrations
INS_MPU6K_FILTER | 0 | 1 | unknown | | false |  | Coming soon
INS_PRODUCT_ID | 0 | 1 | list | 0:Unknown 256:unused 1:unused 257:Linux 2:unused 3:SITL 4:PX4v1 5:PX4v2 88:unused  | false |  | Which type of IMU is installed (MAV_PARAM_GROUP.ARDUCOPTER,read-only).
LAND_REPOSITION | 1 | 1 | list | 0:NoRepositioning 1:Repositioning  | false |  | Enables user input during LAND mode, the landing phase of RTL, and auto mode landings.
LAND_SPEED | 50 | 1 | cm/s | 30 200 | false |  | The descent speed for the final stage of landing in CENTIMETER/s
LOITER_LAT_D | 0 | 1 | unknown | | false |  | Coming soon
LOITER_LAT_I | 0.5 | 1 | unknown | | false |  | Coming soon
LOITER_LAT_IMAX | 1000 | 1 | unknown | | false |  | Coming soon
LOITER_LAT_P | 1 | 1 | unknown | | false |  | Coming soon
LOITER_LON_D | 0 | 1 | unknown | | false |  | Coming soon
LOITER_LON_I | 0.5 | 1 | unknown | | false |  | Coming soon
LOITER_LON_IMAX | 1000 | 1 | unknown | | false |  | Coming soon
LOITER_LON_P | 1 | 1 | unknown | | false |  | Coming soon
MAG_ENABLE | 1 | 1 | list | 0:Disabled 1:Enabled  | false |  | Setting this to Enabled(MAV_PARAM_GROUP.ARDUCOPTER,1) will enable the compass. Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will disable the compass
MIS_RESTART | 0 | 1 | list | 0:ResumeMission 1:RestartMission  | false |  | Controls mission starting point when entering Auto mode (MAV_PARAM_GROUP.ARDUCOPTER,either restart from beginning of mission or resume from last command run)
MIS_TOTAL | 6 | 1 |  | 0 32766 | true | Total mission commands | The number of mission mission items that has been loaded by the ground station. Do not change this manually.
MNT_ANGMAX_PAN | 4500 | 1 | centidegree | -18000 17999 | false |  | Maximum physical pan (MAV_PARAM_GROUP.ARDUCOPTER,yaw) angular position of the mount
MNT_ANGMAX_ROL | 4500 | 1 | centidegree | -18000 17999 | false |  | Maximum physical roll angular position of the mount
MNT_ANGMAX_TIL | 0 | 1 | centidegree | -18000 17999 | false |  | Maximum physical tilt (MAV_PARAM_GROUP.ARDUCOPTER,pitch) angular position of the mount
MNT_ANGMIN_PAN | -4500 | 1 | centidegree | -18000 17999 | false |  | Minimum physical pan (MAV_PARAM_GROUP.ARDUCOPTER,yaw) angular position of mount.
MNT_ANGMIN_ROL | -4500 | 1 | centidegree | -18000 17999 | false |  | Minimum physical roll angular position of mount.
MNT_ANGMIN_TIL | -9000 | 1 | centidegree | -18000 17999 | false |  | Minimum physical tilt (MAV_PARAM_GROUP.ARDUCOPTER,pitch) angular position of mount.
MNT_CONTROL_X | 0 | 1 | unknown | | false |  | Coming soon
MNT_CONTROL_Y | 0 | 1 | unknown | | false |  | Coming soon
MNT_CONTROL_Z | 0 | 1 | unknown | | false |  | Coming soon
MNT_JSTICK_SPD | 0 | 1 | unknown | 0 100 | false |  | 0 for position control, small for low speeds, 100 for max speed. A good general value is 10 which gives a movement speed of 3 degrees per second.
MNT_MODE | 3 | 1 | unknown | | false |  | Coming soon
MNT_NEUTRAL_X | 0 | 1 | degree | -180.0 179.99 | false |  | Mount roll angle when in neutral position
MNT_NEUTRAL_Y | 0 | 1 | degree | -180.0 179.99 | false |  | Mount tilt/pitch angle when in neutral position
MNT_NEUTRAL_Z | 0 | 1 | degree | -180.0 179.99 | false |  | Mount pan/yaw angle when in neutral position
MNT_RC_IN_PAN | 0 | 1 | list | 0:Disabled 5:RC5 6:RC6 7:RC7 8:RC8 9:RC9 10:RC10 11:RC11 12:RC12  | false |  | 0 for none, any other for the RC channel to be used to control pan (MAV_PARAM_GROUP.ARDUCOPTER,yaw) movements
MNT_RC_IN_ROLL | 0 | 1 | list | 0:Disabled 5:RC5 6:RC6 7:RC7 8:RC8 9:RC9 10:RC10 11:RC11 12:RC12  | false |  | 0 for none, any other for the RC channel to be used to control roll movements
MNT_RC_IN_TILT | 6 | 1 | list | 0:Disabled 5:RC5 6:RC6 7:RC7 8:RC8 9:RC9 10:RC10 11:RC11 12:RC12  | false |  | 0 for none, any other for the RC channel to be used to control tilt (MAV_PARAM_GROUP.ARDUCOPTER,pitch) movements
MNT_RETRACT_X | 0 | 1 | degree | -180.0 179.99 | false |  | Mount roll angle when in retracted position
MNT_RETRACT_Y | 0 | 1 | degree | -180.0 179.99 | false |  | Mount tilt/pitch angle when in retracted position
MNT_RETRACT_Z | 0 | 1 | degree | -180.0 179.99 | false |  | Mount yaw/pan angle when in retracted position
MNT_STAB_PAN | 0 | 1 | list | 0:Disabled 1:Enabled  | false |  | enable pan/yaw stabilisation relative to Earth
MNT_STAB_ROLL | 0 | 1 | list | 0:Disabled 1:Enabled  | false |  | enable roll stabilisation relative to Earth
MNT_STAB_TILT | 0 | 1 | list | 0:Disabled 1:Enabled  | false |  | enable tilt/pitch stabilisation relative to Earth
MOT_SPIN_ARMED | 90 | 1 | unknown | | false |  | Coming soon
MOT_TCRV_ENABLE | 1 | 1 | unknown | | false |  | Coming soon
MOT_TCRV_MAXPCT | 93 | 1 | unknown | | false |  | Coming soon
MOT_TCRV_MIDPCT | 52 | 1 | unknown | | false |  | Coming soon
OF_PIT_D | 0.12 | 1 | unknown | | false |  | Coming soon
OF_PIT_I | 0.5 | 1 | unknown | | false |  | Coming soon
OF_PIT_IMAX | 100 | 1 | unknown | | false |  | Coming soon
OF_PIT_P | 2.5 | 1 | unknown | | false |  | Coming soon
OF_RLL_D | 0.12 | 1 | unknown | | false |  | Coming soon
OF_RLL_I | 0.5 | 1 | unknown | | false |  | Coming soon
OF_RLL_IMAX | 100 | 1 | unknown | | false |  | Coming soon
OF_RLL_P | 2.5 | 1 | unknown | | false |  | Coming soon
PHLD_BRAKE_ANGLE | 3000 | 1 | centidegree | 2000 4500 | false |  | PosHold flight mode's max lean angle during braking in centi-degrees
PHLD_BRAKE_RATE | 8 | 1 | deg/s | 4 12 | false | PosHold braking rate | PosHold flight mode's rotation rate during braking in deg/sec
PILOT_ACCEL_Z | 250 | 1 | cm/s/s | 50 500 | false |  | The vertical acceleration used when pilot is controlling the altitude
PILOT_VELZ_MAX | 250 | 1 | cm/s | 50 500 | false |  | The maximum vertical velocity the pilot may request in CENTIMETER/s
POSCON_THR_HOVER | 724 | 1 | unknown | | false |  | Coming soon
RATE_PIT_D | 0.0055 | 1 | unknown | | false |  | Coming soon
RATE_PIT_I | 0.07999999 | 1 | unknown | | false |  | Coming soon
RATE_PIT_IMAX | 1000 | 1 | unknown | | false |  | Coming soon
RATE_PIT_P | 0.07999999 | 1 | unknown | | false |  | Coming soon
RATE_RLL_D | 0.003 | 1 | unknown | | false |  | Coming soon
RATE_RLL_I | 0.08499999 | 1 | unknown | | false |  | Coming soon
RATE_RLL_IMAX | 1000 | 1 | unknown | | false |  | Coming soon
RATE_RLL_P | 0.08499999 | 1 | unknown | | false |  | Coming soon
RATE_YAW_D | 0.003 | 1 | unknown | | false |  | Coming soon
RATE_YAW_I | 0.02 | 1 | unknown | | false |  | Coming soon
RATE_YAW_IMAX | 1000 | 1 | unknown | | false |  | Coming soon
RATE_YAW_P | 0.17 | 1 | unknown | | false |  | Coming soon
RC_FEEL_RP | 100 | 1 | list | 0:Standard 50:Medium 100:VeryCrisp 1000:VerySoft 25:Soft 75:Crisp  | false |  | RC feel for roll/pitch which controls vehicle response to user input with 0 being extremely soft and 100 being crisp
RC_SPEED | 490 | 1 | Hz | 50 490 | false |  | This is the speed in Hertz that your ESCs will receive updates
RC1_DZ | 30 | 1 | PWM | 0 200 | false |  | dead zone around trim or bottom
RC1_MAX | 1976 | 1 | PWM | 800 2200 | false |  | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC1_MIN | 998 | 1 | PWM | 800 2200 | false |  | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC1_REV | 1 | 1 | list | -1:Reversed 1:Normal  | false |  | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC1_TRIM | 1483 | 1 | PWM | 800 2200 | false |  | RC trim (MAV_PARAM_GROUP.ARDUCOPTER,neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC10_DZ | 0 | 1 | PWM | 0 200 | false |  | dead zone around trim or bottom
RC10_FUNCTION | 0 | 1 | list | 0:Disabled 1:RCPassThru 2:Flap 3:Flap_auto 4:Aileron 6:mount_pan 7:mount_tilt 8:mount_roll 9:mount_open 10:camera_trigger 11:release 12:mount2_pan 13:mount2_tilt 14:mount2_roll 15:mount2_open 16:DifferentialSpoiler1 17:DifferentialSpoiler2 18:AileronWithInput 19:Elevator 20:ElevatorWithInput 21:Rudder 24:Flaperon1 25:Flaperon2 26:GroundSteering 27:Parachute 28:EPM 29:LandingGear 30:EngineRunEnable 31:HeliRSC 32:HeliTailRSC 33:Motor1 34:Motor2 35:Motor3 36:Motor4 37:Motor5 38:Motor6 39:Motor7 40:Motor8 51:RCIN1 52:RCIN2 53:RCIN3 54:RCIN4 55:RCIN5 56:RCIN6 57:RCIN7 58:RCIN8 59:RCIN9 60:RCIN10 61:RCIN11 62:RCIN12 63:RCIN13 64:RCIN14 65:RCIN15 66:RCIN16 67:Ignition 68:Choke 69:Starter 70:Throttle  | false |  | Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
RC10_MAX | 1900 | 1 | PWM | 800 2200 | false |  | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC10_MIN | 1100 | 1 | PWM | 800 2200 | false |  | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC10_REV | 1 | 1 | list | -1:Reversed 1:Normal  | false |  | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC10_TRIM | 0 | 1 | PWM | 800 2200 | false |  | RC trim (MAV_PARAM_GROUP.ARDUCOPTER,neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC11_DZ | 0 | 1 | PWM | 0 200 | false |  | dead zone around trim or bottom
RC11_FUNCTION | 0 | 1 | list | 0:Disabled 1:RCPassThru 2:Flap 3:Flap_auto 4:Aileron 6:mount_pan 7:mount_tilt 8:mount_roll 9:mount_open 10:camera_trigger 11:release 12:mount2_pan 13:mount2_tilt 14:mount2_roll 15:mount2_open 16:DifferentialSpoiler1 17:DifferentialSpoiler2 18:AileronWithInput 19:Elevator 20:ElevatorWithInput 21:Rudder 24:Flaperon1 25:Flaperon2 26:GroundSteering 27:Parachute 28:EPM 29:LandingGear 30:EngineRunEnable 31:HeliRSC 32:HeliTailRSC 33:Motor1 34:Motor2 35:Motor3 36:Motor4 37:Motor5 38:Motor6 39:Motor7 40:Motor8 51:RCIN1 52:RCIN2 53:RCIN3 54:RCIN4 55:RCIN5 56:RCIN6 57:RCIN7 58:RCIN8 59:RCIN9 60:RCIN10 61:RCIN11 62:RCIN12 63:RCIN13 64:RCIN14 65:RCIN15 66:RCIN16 67:Ignition 68:Choke 69:Starter 70:Throttle  | false |  | Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
RC11_MAX | 1900 | 1 | PWM | 800 2200 | false |  | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC11_MIN | 1100 | 1 | PWM | 800 2200 | false |  | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC11_REV | 1 | 1 | list | -1:Reversed 1:Normal  | false |  | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC11_TRIM | 0 | 1 | PWM | 800 2200 | false |  | RC trim (MAV_PARAM_GROUP.ARDUCOPTER,neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC2_DZ | 30 | 1 | PWM | 0 200 | false |  | dead zone around trim or bottom
RC2_MAX | 1983 | 1 | PWM | 800 2200 | false |  | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC2_MIN | 996 | 1 | PWM | 800 2200 | false |  | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC2_REV | 1 | 1 | list | -1:Reversed 1:Normal  | false |  | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC2_TRIM | 1492 | 1 | PWM | 800 2200 | false |  | RC trim (MAV_PARAM_GROUP.ARDUCOPTER,neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC3_DZ | 30 | 1 | PWM | 0 200 | false |  | dead zone around trim or bottom
RC3_MAX | 1982 | 1 | PWM | 800 2200 | false |  | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC3_MIN | 996 | 1 | PWM | 800 2200 | false |  | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC3_REV | 1 | 1 | list | -1:Reversed 1:Normal  | false |  | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC3_TRIM | 1000 | 1 | PWM | 800 2200 | false |  | RC trim (MAV_PARAM_GROUP.ARDUCOPTER,neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC4_DZ | 40 | 1 | PWM | 0 200 | false |  | dead zone around trim or bottom
RC4_MAX | 1981 | 1 | PWM | 800 2200 | false |  | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC4_MIN | 992 | 1 | PWM | 800 2200 | false |  | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC4_REV | 1 | 1 | list | -1:Reversed 1:Normal  | false |  | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC4_TRIM | 1486 | 1 | PWM | 800 2200 | false |  | RC trim (MAV_PARAM_GROUP.ARDUCOPTER,neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC5_DZ | 0 | 1 | PWM | 0 200 | false |  | dead zone around trim or bottom
RC5_FUNCTION | 0 | 1 | list | 0:Disabled 1:RCPassThru 2:Flap 3:Flap_auto 4:Aileron 6:mount_pan 7:mount_tilt 8:mount_roll 9:mount_open 10:camera_trigger 11:release 12:mount2_pan 13:mount2_tilt 14:mount2_roll 15:mount2_open 16:DifferentialSpoiler1 17:DifferentialSpoiler2 18:AileronWithInput 19:Elevator 20:ElevatorWithInput 21:Rudder 24:Flaperon1 25:Flaperon2 26:GroundSteering 27:Parachute 28:EPM 29:LandingGear 30:EngineRunEnable 31:HeliRSC 32:HeliTailRSC 33:Motor1 34:Motor2 35:Motor3 36:Motor4 37:Motor5 38:Motor6 39:Motor7 40:Motor8 51:RCIN1 52:RCIN2 53:RCIN3 54:RCIN4 55:RCIN5 56:RCIN6 57:RCIN7 58:RCIN8 59:RCIN9 60:RCIN10 61:RCIN11 62:RCIN12 63:RCIN13 64:RCIN14 65:RCIN15 66:RCIN16 67:Ignition 68:Choke 69:Starter 70:Throttle  | false |  | Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
RC5_MAX | 1982 | 1 | PWM | 800 2200 | false |  | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC5_MIN | 992 | 1 | PWM | 800 2200 | false |  | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC5_REV | 1 | 1 | list | -1:Reversed 1:Normal  | false |  | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC5_TRIM | 993 | 1 | PWM | 800 2200 | false |  | RC trim (MAV_PARAM_GROUP.ARDUCOPTER,neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC6_DZ | 0 | 1 | PWM | 0 200 | false |  | dead zone around trim or bottom
RC6_FUNCTION | 0 | 1 | list | 0:Disabled 1:RCPassThru 2:Flap 3:Flap_auto 4:Aileron 6:mount_pan 7:mount_tilt 8:mount_roll 9:mount_open 10:camera_trigger 11:release 12:mount2_pan 13:mount2_tilt 14:mount2_roll 15:mount2_open 16:DifferentialSpoiler1 17:DifferentialSpoiler2 18:AileronWithInput 19:Elevator 20:ElevatorWithInput 21:Rudder 24:Flaperon1 25:Flaperon2 26:GroundSteering 27:Parachute 28:EPM 29:LandingGear 30:EngineRunEnable 31:HeliRSC 32:HeliTailRSC 33:Motor1 34:Motor2 35:Motor3 36:Motor4 37:Motor5 38:Motor6 39:Motor7 40:Motor8 51:RCIN1 52:RCIN2 53:RCIN3 54:RCIN4 55:RCIN5 56:RCIN6 57:RCIN7 58:RCIN8 59:RCIN9 60:RCIN10 61:RCIN11 62:RCIN12 63:RCIN13 64:RCIN14 65:RCIN15 66:RCIN16 67:Ignition 68:Choke 69:Starter 70:Throttle  | false |  | Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
RC6_MAX | 1985 | 1 | PWM | 800 2200 | false |  | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC6_MIN | 992 | 1 | PWM | 800 2200 | false |  | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC6_REV | 1 | 1 | list | -1:Reversed 1:Normal  | false |  | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC6_TRIM | 992 | 1 | PWM | 800 2200 | false |  | RC trim (MAV_PARAM_GROUP.ARDUCOPTER,neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC7_DZ | 0 | 1 | PWM | 0 200 | false |  | dead zone around trim or bottom
RC7_FUNCTION | 0 | 1 | list | 0:Disabled 1:RCPassThru 2:Flap 3:Flap_auto 4:Aileron 6:mount_pan 7:mount_tilt 8:mount_roll 9:mount_open 10:camera_trigger 11:release 12:mount2_pan 13:mount2_tilt 14:mount2_roll 15:mount2_open 16:DifferentialSpoiler1 17:DifferentialSpoiler2 18:AileronWithInput 19:Elevator 20:ElevatorWithInput 21:Rudder 24:Flaperon1 25:Flaperon2 26:GroundSteering 27:Parachute 28:EPM 29:LandingGear 30:EngineRunEnable 31:HeliRSC 32:HeliTailRSC 33:Motor1 34:Motor2 35:Motor3 36:Motor4 37:Motor5 38:Motor6 39:Motor7 40:Motor8 51:RCIN1 52:RCIN2 53:RCIN3 54:RCIN4 55:RCIN5 56:RCIN6 57:RCIN7 58:RCIN8 59:RCIN9 60:RCIN10 61:RCIN11 62:RCIN12 63:RCIN13 64:RCIN14 65:RCIN15 66:RCIN16 67:Ignition 68:Choke 69:Starter 70:Throttle  | false |  | Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
RC7_MAX | 1900 | 1 | PWM | 800 2200 | false |  | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC7_MIN | 1100 | 1 | PWM | 800 2200 | false |  | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC7_REV | 1 | 1 | list | -1:Reversed 1:Normal  | false |  | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC7_TRIM | 1498 | 1 | PWM | 800 2200 | false |  | RC trim (MAV_PARAM_GROUP.ARDUCOPTER,neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC8_DZ | 0 | 1 | PWM | 0 200 | false |  | dead zone around trim or bottom
RC8_FUNCTION | 0 | 1 | list | 0:Disabled 1:RCPassThru 2:Flap 3:Flap_auto 4:Aileron 6:mount_pan 7:mount_tilt 8:mount_roll 9:mount_open 10:camera_trigger 11:release 12:mount2_pan 13:mount2_tilt 14:mount2_roll 15:mount2_open 16:DifferentialSpoiler1 17:DifferentialSpoiler2 18:AileronWithInput 19:Elevator 20:ElevatorWithInput 21:Rudder 24:Flaperon1 25:Flaperon2 26:GroundSteering 27:Parachute 28:EPM 29:LandingGear 30:EngineRunEnable 31:HeliRSC 32:HeliTailRSC 33:Motor1 34:Motor2 35:Motor3 36:Motor4 37:Motor5 38:Motor6 39:Motor7 40:Motor8 51:RCIN1 52:RCIN2 53:RCIN3 54:RCIN4 55:RCIN5 56:RCIN6 57:RCIN7 58:RCIN8 59:RCIN9 60:RCIN10 61:RCIN11 62:RCIN12 63:RCIN13 64:RCIN14 65:RCIN15 66:RCIN16 67:Ignition 68:Choke 69:Starter 70:Throttle  | false |  | Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
RC8_MAX | 1900 | 1 | PWM | 800 2200 | false |  | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC8_MIN | 1100 | 1 | PWM | 800 2200 | false |  | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC8_REV | 1 | 1 | list | -1:Reversed 1:Normal  | false |  | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC8_TRIM | 1498 | 1 | PWM | 800 2200 | false |  | RC trim (MAV_PARAM_GROUP.ARDUCOPTER,neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RCMAP_PITCH | 2 | 1 |  | | false |  | Pitch channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Pitch is normally on channel 2, but you can move it to any channel with this parameter. Reboot is required for changes to take effect.
RCMAP_ROLL | 1 | 1 |  | | false |  | Roll channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Roll is normally on channel 1, but you can move it to any channel with this parameter. Reboot is required for changes to take effect.
RCMAP_THROTTLE | 3 | 1 |  | | false |  | Throttle channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Throttle is normally on channel 3, but you can move it to any channel with this parameter. Warning APM 2.X: Changing the throttle channel could produce unexpected fail-safe results if connection between receiver and on-board PPM Encoder is lost. Disabling on-board PPM Encoder is recommended. Reboot is required for changes to take effect.
RCMAP_YAW | 4 | 1 |  | | false |  | Yaw channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Yaw (MAV_PARAM_GROUP.ARDUCOPTER,also known as rudder) is normally on channel 4, but you can move it to any channel with this parameter. Reboot is required for changes to take effect.
RELAY_PIN | 13 | 1 | list | -1:Disabled 13:APM2 A9 pin 47:APM1 relay 111:PX4 FMU Relay1 112:PX4 FMU Relay2 113:PX4IO Relay1 50:Pixhawk AUXOUT1 114:PX4IO Relay2 51:Pixhawk AUXOUT2 115:PX4IO ACC1 52:Pixhawk AUXOUT3 116:PX4IO ACC2 53:Pixhawk AUXOUT4 54:Pixhawk AUXOUT5 55:Pixhawk AUXOUT6  | false |  | Digital pin number for first relay control. This is the pin used for camera control.
RELAY_PIN2 | -1 | 1 | list | -1:Disabled 13:APM2 A9 pin 47:APM1 relay 111:PX4 FMU Relay1 112:PX4 FMU Relay2 113:PX4IO Relay1 50:Pixhawk AUXOUT1 114:PX4IO Relay2 51:Pixhawk AUXOUT2 115:PX4IO ACC1 52:Pixhawk AUXOUT3 116:PX4IO ACC2 53:Pixhawk AUXOUT4 54:Pixhawk AUXOUT5 55:Pixhawk AUXOUT6  | false |  | Digital pin number for 2nd relay control.
RNGFND_FUNCTION | 0 | 1 | list | 0:Linear 1:Inverted 2:Hyperbolic  | false |  | Control over what function is used to calculate distance. For a linear function, the distance is (MAV_PARAM_GROUP.ARDUCOPTER,voltage-offset)*scaling. For a inverted function the distance is (MAV_PARAM_GROUP.ARDUCOPTER,offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(MAV_PARAM_GROUP.ARDUCOPTER,voltage-offset). The functions return the distance in meters.
RNGFND_GAIN | 0.8 | 1 | unknown | 0.01 2.0 | false |  | Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter
RNGFND_MAX_CM | 700 | 1 | cm | | false |  | Maximum distance in centimeters that rangefinder can reliably read
RNGFND_MIN_CM | 20 | 1 | cm | | false |  | Minimum distance in centimeters that rangefinder can reliably read
RNGFND_OFFSET | 0 | 1 | volt | | false |  | Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM and I2C Lidars
RNGFND_PIN | -1 | 1 | list | -1:NotUsed 0:APM2-A0 64:APM1-airspeed port 1:APM2-A1 2:APM2-A2 3:APM2-A3 4:APM2-A4 5:APM2-A5 6:APM2-A6 7:APM2-A7 8:APM2-A8 9:APM2-A9 11:PX4-airspeed port 15:Pixhawk-airspeed port  | false |  | Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
RNGFND_RMETRIC | 1 | 1 | list | 0:No 1:Yes  | false |  | This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (MAV_PARAM_GROUP.ARDUCOPTER,such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
RNGFND_SETTLE_MS | 0 | 1 | unknown | | false |  | Coming soon
RNGFND_STOP_PIN | -1 | 1 | list | -1:NotUsed 112:PX4 FMU Relay2 113:PX4IO,Relay1 50:Pixhawk AUXOUT1 114:PX4IO Relay2 51:Pixhawk AUXOUT2 115:PX4IO ACC1 52:Pixhawk AUXOUT3,53:Pixhawk,AUXOUT4,54:Pixhawk AUXOUT5 116:PX4IO ACC2 55:Pixhawk AUXOUT6 111:PX4 FMU Relay1  | false |  | Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
RNGFND_TYPE | 0 | 1 | list | 0:None 1:Analog 2:APM2-MaxbotixI2C 3:APM2-PulsedLightI2C 4:PX4-I2C 5:PX4-PWM 6:BBB-PRU 7:LightWareI2C 8:LightWareSerial 9:Bebop 10:MAVLink 12:LeddarOne  | false |  | What type of rangefinder device that is connected
RSSI_PIN | -1 | 1 | unknown | | false |  | Coming soon
RSSI_RANGE | 5 | 1 | unknown | | false |  | Coming soon
RTL_ALT | 1500 | 1 | cm | 0 8000 | false |  | The minimum relative altitude the model will move to before Returning to Launch. Set to zero to return at current altitude.
RTL_ALT_FINAL | 0 | 1 | cm | -1 1000 | false |  | This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission. Set to zero to land.
RTL_LOIT_TIME | 5000 | 1 | m/s | 0 60000 | false |  | Time (MAV_PARAM_GROUP.ARDUCOPTER,in milliseconds) to loiter above home before beginning final descent
SCHED_DEBUG | 0 | 1 | list | 0:Disabled 2:ShowSlips 3:ShowOverruns  | false | Scheduler debug level |" Set to non-zero to enable scheduler debug messages. When set to show ""Slips"" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table."
SERIAL0_BAUD | 115 | 1 | list | 1:1200 2:2400 19:19200 115:115200 4:4800 500:500000 38:38400 9:9600 57:57600 921:921600 1500:1500000 111:111100  | false |  | The baud rate used on the USB console. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
SERIAL1_BAUD | 57 | 1 | list | 1:1200 2:2400 19:19200 115:115200 4:4800 500:500000 38:38400 9:9600 57:57600 921:921600 1500:1500000 111:111100  | false |  | The baud rate used on the Telem1 port. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
SIMPLE | 18 | 1 | unknown | | false |  | Bitmask which holds which flight modes use simple heading mode (MAV_PARAM_GROUP.ARDUCOPTER,eg bit 0 = 1 means Flight Mode 0 uses simple mode)
SR0_EXT_STAT | 2 | 1 | Hz | 0 10 | false |  | Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
SR0_EXTRA1 | 4 | 1 | Hz | 0 10 | false |  | Stream rate of ATTITUDE and SIMSTATE (MAV_PARAM_GROUP.ARDUCOPTER,SITL only) to ground station
SR0_EXTRA2 | 4 | 1 | Hz | 0 10 | false |  | Stream rate of VFR_HUD to ground station
SR0_EXTRA3 | 2 | 1 | Hz | 0 10 | false |  | Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
SR0_PARAMS | 10 | 1 | Hz | 0 10 | false |  | Stream rate of PARAM_VALUE to ground station
SR0_POSITION | 2 | 1 | Hz | 0 10 | false |  | Stream rate of GLOBAL_POSITION_INT to ground station
SR0_RAW_CTRL | 1 | 1 | Hz | 0 10 | false |  | Stream rate of RC_CHANNELS_SCALED (MAV_PARAM_GROUP.ARDUCOPTER,HIL only) to ground station
SR0_RAW_SENS | 2 | 1 | Hz | 0 10 | false |  | Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
SR0_RC_CHAN | 2 | 1 | Hz | 0 10 | false |  | Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station
SR1_EXT_STAT | 2 | 1 | Hz | 0 10 | false |  | Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
SR1_EXTRA1 | 2 | 1 | Hz | 0 10 | false |  | Stream rate of ATTITUDE and SIMSTATE (MAV_PARAM_GROUP.ARDUCOPTER,SITL only) to ground station
SR1_EXTRA2 | 2 | 1 | Hz | 0 10 | false |  | Stream rate of VFR_HUD to ground station
SR1_EXTRA3 | 2 | 1 | Hz | 0 10 | false |  | Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
SR1_PARAMS | 0 | 1 | Hz | 0 10 | false |  | Stream rate of PARAM_VALUE to ground station
SR1_POSITION | 2 | 1 | Hz | 0 10 | false |  | Stream rate of GLOBAL_POSITION_INT to ground station
SR1_RAW_CTRL | 2 | 1 | Hz | 0 10 | false |  | Stream rate of RC_CHANNELS_SCALED (MAV_PARAM_GROUP.ARDUCOPTER,HIL only) to ground station
SR1_RAW_SENS | 2 | 1 | Hz | 0 10 | false |  | Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
SR1_RC_CHAN | 2 | 1 | Hz | 0 10 | false |  | Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station
STB_PIT_P | 0.016 | 1 | unknown | | false |  | Coming soon
STB_RLL_P | 0.016 | 1 | unknown | | false |  | Coming soon
STB_YAW_P | 4 | 1 | unknown | | false |  | Coming soon
SUPER_SIMPLE | 5 | 1 | list | 0:Disabled 1:Mode1 2:Mode2 3:Mode1+2 4:Mode3 5:Mode1+3 6:Mode2+3 7:Mode1+2+3 8:Mode4 9:Mode1+4 10:Mode2+4 11:Mode1+2+4 12:Mode3+4 13:Mode1+3+4 14:Mode2+3+4 15:Mode1+2+3+4 16:Mode5 17:Mode1+5 18:Mode2+5 19:Mode1+2+5 20:Mode3+5 21:Mode1+3+5 22:Mode2+3+5 23:Mode1+2+3+5 24:Mode4+5 25:Mode1+4+5 26:Mode2+4+5 27:Mode1+2+4+5 28:Mode3+4+5 29:Mode1+3+4+5 30:Mode2+3+4+5 31:Mode1+2+3+4+5 32:Mode6 33:Mode1+6 34:Mode2+6 35:Mode1+2+6 36:Mode3+6 37:Mode1+3+6 38:Mode2+3+6 39:Mode1+2+3+6 40:Mode4+6 41:Mode1+4+6 42:Mode2+4+6 43:Mode1+2+4+6 44:Mode3+4+6 45:Mode1+3+4+6 46:Mode2+3+4+6 47:Mode1+2+3+4+6 48:Mode5+6 49:Mode1+5+6 50:Mode2+5+6 51:Mode1+2+5+6 52:Mode3+5+6 53:Mode1+3+5+6 54:Mode2+3+5+6 55:Mode1+2+3+5+6 56:Mode4+5+6 57:Mode1+4+5+6 58:Mode2+4+5+6 59:Mode1+2+4+5+6 60:Mode3+4+5+6 61:Mode1+3+4+5+6 62:Mode2+3+4+5+6 63:Mode1+2+3+4+5+6  | false |  | Bitmask to enable Super Simple mode for some flight modes. Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will disable Super Simple Mode
SYSID_MYGCS | 255 | 1 | list | 252:AP Planner 2 255:Mission Planner and DroidPlanner  | false | My ground station number | Allows restricting radio overrides to only come from my ground station
SYSID_SW_MREV | 120 | 1 | unknown | | true | Eeprom format version number | This value is incremented when changes are made to the eeprom format
SYSID_SW_TYPE | 10 | 1 | list | 0:ArduPlane 4:AntennaTracker 20:Rover 10:Copter  | false |  | This is used by the ground station to recognise the software type (MAV_PARAM_GROUP.ARDUCOPTER,eg ArduPlane vs ArduCopter)
SYSID_THISMAV | 1 | 1 | unknown | 1 255 | false |  | Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
TELEM_DELAY | 0 | 1 | second | 0 30 | false |  | The amount of time (MAV_PARAM_GROUP.ARDUCOPTER,in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
THR_ACCEL_D | 0 | 1 | unknown | | false |  | Coming soon
THR_ACCEL_I | 1 | 1 | unknown | | false |  | Coming soon
THR_ACCEL_IMAX | 800 | 1 | unknown | | false |  | Coming soon
THR_ACCEL_P | 0.5 | 1 | unknown | | false |  | Coming soon
THR_ALT_P | 1 | 1 | unknown | | false |  | Coming soon
THR_DZ | 100 | 1 | PWM | 0 300 | false |  | The deadzone above and below mid throttle. Used in AltHold, Loiter, PosHold flight modes
THR_MAX | 1000 | 1 | unknown | | false |  | Coming soon
THR_MID | 510 | 1 | unknown | | false |  | Coming soon
THR_MIN | 130 | 1 | unknown | | false |  | Coming soon
THR_RATE_P | 6 | 1 | unknown | | false |  | Coming soon
TRIM_THROTTLE | 724 | 1 | unknown | | false |  | Coming soon
TUNE | 1 | 1 | list | 0:None 1:Stab_Roll/Pitch_kP 3:Stab_Yaw_kP 4:Rate_Roll/Pitch_kP 5:Rate_Roll/Pitch_kI 6:Rate_Yaw_kP 7:Throttle_Rate_kP 10:WP_Speed 12:Loiter_Pos_kP 13:Heli_Ext_Gyro 14:Altitude_Hold_kP 17:OF_Loiter_kP 18:OF_Loiter_kI 19:OF_Loiter_kD 21:Rate_Roll/Pitch_kD 22:Velocity_XY_kP 25:Acro_RollPitch_kP 26:Rate_Yaw_kD 28:Velocity_XY_kI 34:Throttle_Accel_kP 35:Throttle_Accel_kI 36:Throttle_Accel_kD 38:Declination 39:Circle_Rate 40:Acro_Yaw_kP 41:RangeFinder_Gain 42:Loiter_Speed 46:Rate_Pitch_kP 47:Rate_Pitch_kI 48:Rate_Pitch_kD 49:Rate_Roll_kP 50:Rate_Roll_kI 51:Rate_Roll_kD 52:Rate_Pitch_FF 53:Rate_Roll_FF 54:Rate_Yaw_FF  | false |  | Controls which parameters (MAV_PARAM_GROUP.ARDUCOPTER,normally PID gains) are being tuned with transmitter's channel 6 knob
TUNE_HIGH | 32 | 1 | unknown | 0 32767 | false |  | The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
TUNE_LOW | 0 | 1 | unknown | 0 32767 | false |  | The minimum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
WP_YAW_BEHAVIOR | 2 | 1 | list | 0:NeverChangeYaw 1:FaceNextWaypoint 2:FaceNextWaypointExceptRTL 3:FaceAlongGPScourse  | false |  | Determines how the autopilot controls the yaw during missions and RTL
WPNAV_ACCEL | 100 | 1 | cm/s/s | 50 500 | false |  | Defines the horizontal acceleration in CENTIMETER/s/s used during missions
WPNAV_ACCEL_Z | 100 | 1 | cm/s/s | 50 500 | false |  | Defines the vertical acceleration in CENTIMETER/s/s used during missions
WPNAV_LOIT_JERK | 1000 | 1 | cm/s/s/s | 500 5000 | false |  | Loiter maximum jerk in CENTIMETER/s/s/s
WPNAV_LOIT_SPEED | 1000 | 1 | cm/s | 20 2000 | false |  | Defines the maximum speed in CENTIMETER/s which the aircraft will travel horizontally while in loiter mode
WPNAV_RADIUS | 200 | 1 | cm | 100 1000 | false |  | Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
WPNAV_SPEED | 500 | 1 | cm/s | 0 2000 | false |  | Defines the speed in CENTIMETER/s which the aircraft will attempt to maintain horizontally during a WP mission
WPNAV_SPEED_DN | 150 | 1 | cm/s | 10 500 | false |  | Defines the speed in CENTIMETER/s which the aircraft will attempt to maintain while descending during a WP mission
WPNAV_SPEED_UP | 250 | 1 | cm/s | 10 1000 | false | Waypoint Climb Speed Target | Defines the speed in CENTIMETER/s which the aircraft will attempt to maintain while climbing during a WP mission


ArduPilot Parameters
Name | Possible Value | Increment | Unit | Range | Read Only | Title | Description
--- | --- | --- | --- | --- | ---| --- | ---
THR_PASS_STAB | 0 | 1 | unknown | -100 100 | false |  | Throttle passthru in stabilize
THR_FAILSAFE | 1 | 1 | list | 0:Disable 1:Enable  | false |  | 
GPS_SBAS_MODE | 2 | 1 | list | 0:Disabled 1:Enable 2:NoChange  | false |  | 
FORMAT_VERSION | 13 | 1 | unknown | 1 255 | false |  | Eeprom format version number
SYSID_SW_TYPE | 0 | 1 | unknown | | false |  | Unknown
SYSID_THISMAV | 1 | 1 | unknown | 1 255 | false |  | MAVLink system ID of this vehicle
SYSID_MYGCS | 255 | 1 | unknown | 1 255 | false |  | Ground station MAVLink system ID
SERIAL0_PROTOCOL | 1 | 1 | list | 1:MAVlink1 2:MAVlink2  | false |  | 
SERIAL0_BAUD | 115 | 1 | list | 0:1200 256:256000 2:2400 4:4800 38:38400 9:9600 460:460800 111:111100 19:19200 115:115200 500:500000 57:57600 921:921600 1500:1500000  | false |  | 
SERIAL1_PROTOCOL | 1 | 1 | list | 1:MAVlink1 2:MAVlink2  | false |  | 
SERIAL1_BAUD | 57 | 1 | list | 0:1200 256:256000 2:2400 4:4800 38:38400 9:9600 460:460800 111:111100 19:19200 115:115200 500:500000 57:57600 921:921600 1500:1500000  | false |  | 
SERIAL2_PROTOCOL | 1 | 1 | list | 1:MAVlink1 2:MAVlink2  | false |  | 
SERIAL2_BAUD | 57 | 1 | list | 0:1200 256:256000 2:2400 4:4800 38:38400 9:9600 460:460800 111:111100 19:19200 115:115200 500:500000 57:57600 921:921600 1500:1500000  | false |  | 
SERIAL3_PROTOCOL | 5 | 1 | list | 1:MAVlink1 2:MAVlink2  | false |  | 
SERIAL3_BAUD | 38 | 1 | list | 0:1200 256:256000 2:2400 4:4800 38:38400 9:9600 460:460800 111:111100 19:19200 115:115200 500:500000 57:57600 921:921600 1500:1500000  | false |  | 
SERIAL4_PROTOCOL | 5 | 1 | list | 1:MAVlink1 2:MAVlink2  | false |  | 
SERIAL4_BAUD | 38 | 1 | list | 0:1200 256:256000 2:2400 4:4800 38:38400 9:9600 460:460800 111:111100 19:19200 115:115200 500:500000 57:57600 921:921600 1500:1500000  | false |  | 
AUTOTUNE_LEVEL | 6 | 1 | unknown | | false |  | Autotune level
TELEM_DELAY | 0 | 1 | unknown | 0 30 | false |  | Telemetry startup delay
GCS_PID_MASK | 0 | 1 | list | 0:Roll 1:Pitch 2:Yaw 3:Steering 4:Landing  | false |  | 
KFF_RDDRMIX | 0.5 | 0.01 | unknown | 0 1 | false |  | Rudder Mix
KFF_THR2PTCH | 0 | 0.01 | unknown | 0 1 | false |  | Throttle to Pitch Mix
STAB_PITCH_DOWN | 2 | 0.01 | unknown | 0 15 | false |  | Low throttle pitch down trim
GLIDE_SLOPE_MIN | 15 | 1 | meter | 0 1000 | false |  | Glide slope minimum
GLIDE_SLOPE_THR | 5 | 1 | unknown | 0 100 | false |  | Glide slope threshold
STICK_MIXING | 1 | 1 | list | 0:Disabled 1:FBWMixing 2:DirectMixing  | false |  | 
SKIP_GYRO_CAL | 0 | 1 | unknown | | false |  | Unknown
AUTO_FBW_STEER | 0 | 1 | list | 0:Disabled 42:Enabled  | false |  | Use FBWA steering in AUTO, 0:Disabled 42:Enabled
TKOFF_THR_MINSPD | 0 | 0.1 | m/s | 0 30 | false |  | Takeoff throttle min speed
TKOFF_THR_MINACC | 0 | 0.1 | m/s/s | 0 30 | false |  | Takeoff throttle min acceleration
TKOFF_THR_DELAY | 2 | 1 | unknown | 0 127 | false |  | Takeoff throttle delay (MAV_PARAM_GROUP.ARDUPLANE,Increment 1 deciseconds)
TKOFF_TDRAG_ELEV | 0 | 1 | unknown | | false |  | FBWA taildragger channel
TKOFF_TDRAG_SPD1 | 0 | 0.1 | m/s | 0 30 | false |  | Takeoff tail dragger speed1
TKOFF_ROTATE_SPD | 0 | 0.1 | m/s | 0 30 | false |  | Takeoff rotate speed
TKOFF_THR_SLEW | 0 | 1 | unknown | -1 127 | false |  | Takeoff throttle slew rate
TKOFF_FLAP_PCNT | 0 | 1 | unknown | 0 100 | false |  | Takeoff flap percentage
FBWA_TDRAG_CHAN | 0 | 1 | unknown | | false |  | FBWA taildragger channel
LEVEL_ROLL_LIMIT | 5 | 1 | degree | 0 45 | false |  | Level flight roll limit
LAND_PITCH_CD | 0 | 1 | unknown | | false |  | Landing Pitch
LAND_FLARE_ALT | 3 | 0.1 | degree | | false |  | Landing flare altitude
LAND_FLARE_SEC | 2 | 0.1 | degree | | false |  | Landing flare time
LAND_DISARMDELAY | 20 | 1 | second | 0 127 | false |  | Landing disarm delay
NAV_CONTROLLER | 1 | 1 | list | 0:Default 1:L1Controller  | false |  | Navigation controller selection
ALT_MIX | 1 | 1 | unknown | | false |  | Not Yet
ALT_CTRL_ALG | 0 | 1 | list | 0:Automatic  | false |  | Altitude control algorithm
ALT_OFFSET | 0 | 1 | meter | -32767 32767 | false |  | Altitude offset
WP_RADIUS | 90 | 1 | meter | 1 32767 | false |  | Waypoint Radius
WP_MAX_RADIUS | 0 | 1 | meter | 0 32767 | false |  | Waypoint Maximum Radius
WP_LOITER_RAD | 60 | 1 | unknown | -32767 32767 | false |  | Waypoint Loiter Radius
FENCE_ACTION | 0 | 1 | list | 0:None 1:GuidedMode 2:ReportOnly 3:GuidedModeThrPass 4:RTL_Mode  | false |  | Action on geofence breach
FENCE_TOTAL | 0 | 1 | unknown | | false |  | Fence Total
FENCE_CHANNEL | 0 | 1 | unknown | | false |  | Fence Channel
FENCE_MINALT | 0 | 1 | meter | 0 32767 | false |  | Fence Minimum Altitude
FENCE_MAXALT | 0 | 1 | meter | 0 32767 | false |  | Fence Maximum Altitude
FENCE_RETALT | 0 | 1 | meter | 0 32767 | false |  | Fence Return Altitude
FENCE_AUTOENABLE | 0 | 1 | list | 0:NoAutoEnable 1:AutoEnable 2:AutoEnableDisableFloorOnly 3:EnableWhenArmed  | false |  | Fence automatic enable
FENCE_RET_RALLY | 0 | 1 | list | 0:FenceReturnPoint 1:NearestRallyPoint  | false |  | Fence Return to Rally
STALL_PREVENTION | 1 | 1 | list | 0:Disabled 1:Enabled  | false |  | Enable stall prevention
ARSPD_FBW_MIN | 9 | 1 | m/s | 5 100 | false |  | Minimum Airspeed
ARSPD_FBW_MAX | 22 | 1 | m/s | 5 100 | false |  | Maximum Airspeed
FBWB_ELEV_REV | 0 | 1 | list | 0:Disabled 1:Enabled  | false |  | Fly By Wire elevator reverse
FBWB_CLIMB_RATE | 2 | 0.1 | m/s | 1 10 | false |  | Fly By Wire B altitude change rate
THR_MIN | 0 | 1 | percent | -100 100 | false |  | Minimum Throttle
THR_MAX | 75 | 1 | percent | 0 100 | false |  | Maximum Throttle
TKOFF_THR_MAX | 0 | 1 | percent | 0 100 | false |  | Maximum Throttle for takeoff
THR_SLEWRATE | 100 | 1 | unknown | 0 127 | false |  | Throttle slew rate
FLAP_SLEWRATE | 75 | 1 | unknown | 0 100 | false |  | Flap slew rate
THR_SUPP_MAN | 0 | 1 | unknown | | false |  | Not Yet
THR_FS_VALUE | 950 | 1 | unknown | | false |  | Not Yet
TRIM_THROTTLE | 45 | 1 | unknown | | false |  | Not Yet
THROTTLE_NUDGE | 1 | 1 | unknown | | false |  | Not Yet
FS_SHORT_ACTN | 0 | 1 | unknown | | false |  | Not Yet
FS_SHORT_TIMEOUT | 1.5 | 1 | unknown | | false |  | Not Yet
FS_LONG_ACTN | 0 | 1 | unknown | | false |  | Not Yet
FS_LONG_TIMEOUT | 5 | 1 | unknown | | false |  | Not Yet
FS_BATT_VOLTAGE | 0 | 1 | unknown | | false |  | Not Yet
FS_BATT_MAH | 0 | 1 | unknown | | false |  | Not Yet
FS_GCS_ENABL | 0 | 1 | unknown | | false |  | Not Yet
FLTMODE_CH | 8 | 1 | unknown | | false |  | Not Yet
FLTMODE1 | 11 | 1 | unknown | | false |  | Not Yet
FLTMODE2 | 11 | 1 | unknown | | false |  | Not Yet
FLTMODE3 | 5 | 1 | unknown | | false |  | Not Yet
FLTMODE4 | 5 | 1 | unknown | | false |  | Not Yet
FLTMODE5 | 0 | 1 | unknown | | false |  | Not Yet
FLTMODE6 | 0 | 1 | unknown | | false |  | Not Yet
INITIAL_MODE | 0 | 1 | unknown | | false |  | Not Yet
LIM_ROLL_CD | 4500 | 1 | unknown | | false |  | Not Yet
LIM_PITCH_MAX | 2000 | 1 | unknown | | false |  | Not Yet
LIM_PITCH_MIN | -2500 | 1 | unknown | | false |  | Not Yet
ACRO_ROLL_RATE | 180 | 1 | unknown | | false |  | Not Yet
ACRO_PITCH_RATE | 180 | 1 | unknown | | false |  | Not Yet
ACRO_LOCKING | 0 | 1 | unknown | | false |  | Not Yet
GROUND_STEER_ALT | 0 | 1 | unknown | | false |  | Not Yet
GROUND_STEER_DPS | 90 | 1 | unknown | | false |  | Not Yet
TRIM_AUTO | 0 | 1 | unknown | | false |  | Not Yet
ELEVON_MIXING | 0 | 1 | unknown | | false |  | Not Yet
ELEVON_REVERSE | 0 | 1 | unknown | | false |  | Not Yet
ELEVON_CH1_REV | 0 | 1 | unknown | | false |  | Not Yet
ELEVON_CH2_REV | 0 | 1 | unknown | | false |  | Not Yet
VTAIL_OUTPUT | 0 | 1 | unknown | | false |  | Not Yet
ELEVON_OUTPUT | 0 | 1 | unknown | | false |  | Not Yet
MIXING_GAIN | 0.5 | 1 | unknown | | false |  | Not Yet
RUDDER_ONLY | 0 | 1 | unknown | | false |  | Not Yet
SYS_NUM_RESETS | 13 | 1 | unknown | | false |  | Not Yet
LOG_BITMASK | 16254 | 1 | unknown | | false |  | Not Yet
RST_SWITCH_CH | 0 | 1 | unknown | | false |  | Not Yet
RST_MISSION_CH | 0 | 1 | unknown | | false |  | Not Yet
TRIM_ARSPD_CM | 1200 | 1 | unknown | | false |  | Not Yet
SCALING_SPEED | 15 | 1 | unknown | | false |  | Not Yet
MIN_GNDSPD_CM | 0 | 1 | unknown | | false |  | Not Yet
TRIM_PITCH_CD | 0 | 1 | unknown | | false |  | Not Yet
ALT_HOLD_RTL | 10000 | 1 | unknown | | false |  | Not Yet
ALT_HOLD_FBWCM | 0 | 1 | unknown | | false |  | Not Yet
MAG_ENABLE | 1 | 1 | unknown | | false |  | Not Yet
FLAP_IN_CHANNEL | 0 | 1 | unknown | | false |  | Not Yet
FLAPERON_OUTPUT | 0 | 1 | unknown | | false |  | Not Yet
FLAP_1_PERCNT | 0 | 1 | unknown | | false |  | Not Yet
FLAP_1_SPEED | 0 | 1 | unknown | | false |  | Not Yet
FLAP_2_PERCNT | 0 | 1 | unknown | | false |  | Not Yet
FLAP_2_SPEED | 0 | 1 | unknown | | false |  | Not Yet
LAND_FLAP_PERCNT | 0 | 1 | unknown | | false |  | Not Yet
RSSI_PIN | -1 | 1 | unknown | | false |  | Not Yet
RSSI_RANGE | 5 | 1 | unknown | | false |  | Not Yet
INVERTEDFLT_CH | 0 | 1 | unknown | | false |  | Not Yet
HIL_SERVOS | 0 | 1 | unknown | | false |  | Not Yet
HIL_ERR_LIMIT | 5 | 1 | unknown | | false |  | Not Yet
RTL_AUTOLAND | 0 | 1 | unknown | | false |  | Not Yet
TRIM_RC_AT_START | 0 | 1 | unknown | | false |  | Not Yet
GND_ABS_PRESS | 50705.5625 | 1 | unknown | | false |  | Not Yet
GND_TEMP | 25 | 1 | unknown | | false |  | Not Yet
GND_ALT_OFFSET | 0 | 1 | unknown | | false |  | Not Yet
GPS_TYPE | 1 | 1 | unknown | | false |  | Not Yet
GPS_NAVFILTER | 8 | 1 | unknown | | false |  | Not Yet
GPS_MIN_ELEV | -100 | 1 | unknown | | false |  | Not Yet
GPS_GNSS_MODE | 0 | 1 | unknown | | false |  | Not Yet
CAM_TRIGG_TYPE | 0 | 1 | unknown | | false |  | Not Yet
CAM_DURATION | 10 | 1 | unknown | | false |  | Not Yet
CAM_SERVO_ON | 1300 | 1 | unknown | | false |  | Not Yet
CAM_SERVO_OFF | 1100 | 1 | unknown | | false |  | Not Yet
CAM_TRIGG_DIST | 0 | 1 | unknown | | false |  | Not Yet
ARMING_REQUIRE | 1 | 1 | unknown | | false |  | Not Yet
ARMING_CHECK | 1 | 1 | unknown | | false |  | Not Yet
ARMING_RUDDER | 1 | 1 | unknown | | false |  | Not Yet
RELAY_PIN | 13 | 1 | unknown | | false |  | Not Yet
RELAY_PIN2 | -1 | 1 | unknown | | false |  | Not Yet
RELAY_PIN3 | -1 | 1 | unknown | | false |  | Not Yet
RELAY_PIN4 | -1 | 1 | unknown | | false |  | Not Yet
RELAY_DEFAULT | 0 | 1 | unknown | | false |  | Not Yet
RNGFND_LANDING | 0 | 1 | unknown | | false |  | Not Yet
RC1_MIN | 1100 | 1 | unknown | | false |  | Not Yet
RC1_TRIM | 1500 | 1 | unknown | | false |  | Not Yet
RC1_MAX | 1900 | 1 | unknown | | false |  | Not Yet
RC1_REV | 1 | 1 | unknown | | false |  | Not Yet
RC1_DZ | 30 | 1 | unknown | | false |  | Not Yet
RC2_MIN | 1100 | 1 | unknown | | false |  | Not Yet
RC2_TRIM | 1500 | 1 | unknown | | false |  | Not Yet
RC2_MAX | 1900 | 1 | unknown | | false |  | Not Yet
RC2_REV | 1 | 1 | unknown | | false |  | Not Yet
RC2_DZ | 30 | 1 | unknown | | false |  | Not Yet
RC3_MIN | 1100 | 1 | unknown | | false |  | Not Yet
RC3_TRIM | 1100 | 1 | unknown | | false |  | Not Yet
RC3_MAX | 1900 | 1 | unknown | | false |  | Not Yet
RC3_REV | 1 | 1 | unknown | | false |  | Not Yet
RC3_DZ | 30 | 1 | unknown | | false |  | Not Yet
RC4_MIN | 1100 | 1 | unknown | | false |  | Not Yet
RC4_TRIM | 1500 | 1 | unknown | | false |  | Not Yet
RC4_MAX | 1900 | 1 | unknown | | false |  | Not Yet
RC4_REV | 1 | 1 | unknown | | false |  | Not Yet
RC4_DZ | 30 | 1 | unknown | | false |  | Not Yet
RC5_MIN | 1100 | 1 | unknown | | false |  | Not Yet
RC5_TRIM | 1500 | 1 | unknown | | false |  | Not Yet
RC5_MAX | 1900 | 1 | unknown | | false |  | Not Yet
RC5_REV | 1 | 1 | unknown | | false |  | Not Yet
RC5_DZ | 0 | 1 | unknown | | false |  | Not Yet
RC5_FUNCTION | 0 | 1 | unknown | | false |  | Not Yet
RC6_MIN | 1100 | 1 | unknown | | false |  | Not Yet
RC6_TRIM | 1500 | 1 | unknown | | false |  | Not Yet
RC6_MAX | 1900 | 1 | unknown | | false |  | Not Yet
RC6_REV | 1 | 1 | unknown | | false |  | Not Yet
RC6_DZ | 0 | 1 | unknown | | false |  | Not Yet
RC6_FUNCTION | 0 | 1 | unknown | | false |  | Not Yet
RC7_MIN | 1100 | 1 | unknown | | false |  | Not Yet
RC7_TRIM | 1500 | 1 | unknown | | false |  | Not Yet
RC7_MAX | 1900 | 1 | unknown | | false |  | Not Yet
RC7_REV | 1 | 1 | unknown | | false |  | Not Yet
RC7_DZ | 0 | 1 | unknown | | false |  | Not Yet
RC7_FUNCTION | 0 | 1 | unknown | | false |  | Not Yet
RC8_MIN | 1100 | 1 | unknown | | false |  | Not Yet
RC8_TRIM | 1500 | 1 | unknown | | false |  | Not Yet
RC8_MAX | 1900 | 1 | unknown | | false |  | Not Yet
RC8_REV | 1 | 1 | unknown | | false |  | Not Yet
RC8_DZ | 0 | 1 | unknown | | false |  | Not Yet
RC8_FUNCTION | 0 | 1 | unknown | | false |  | Not Yet
RC10_MIN | 1100 | 1 | unknown | | false |  | Not Yet
RC10_TRIM | 1500 | 1 | unknown | | false |  | Not Yet
RC10_MAX | 1900 | 1 | unknown | | false |  | Not Yet
RC10_REV | 1 | 1 | unknown | | false |  | Not Yet
RC10_DZ | 0 | 1 | unknown | | false |  | Not Yet
RC10_FUNCTION | 0 | 1 | unknown | | false |  | Not Yet
RC11_MIN | 1100 | 1 | unknown | | false |  | Not Yet
RC11_TRIM | 1500 | 1 | unknown | | false |  | Not Yet
RC11_MAX | 1900 | 1 | unknown | | false |  | Not Yet
RC11_REV | 1 | 1 | unknown | | false |  | Not Yet
RC11_DZ | 0 | 1 | unknown | | false |  | Not Yet
RC11_FUNCTION | 0 | 1 | unknown | | false |  | Not Yet
RLL2SRV_TCONST | 0.5 | 1 | unknown | | false |  | Not Yet
RLL2SRV_P | 0.400000005960465 | 1 | unknown | | false |  | Not Yet
RLL2SRV_D | 0.0199999995529652 | 1 | unknown | | false |  | Not Yet
RLL2SRV_I | 0.0399999991059303 | 1 | unknown | | false |  | Not Yet
RLL2SRV_RMAX | 0 | 1 | unknown | | false |  | Not Yet
RLL2SRV_IMAX | 3000 | 1 | unknown | | false |  | Not Yet
RLL2SRV_FF | 0 | 1 | unknown | | false |  | Not Yet
PTCH2SRV_TCONST | 0.5 | 1 | unknown | | false |  | Not Yet
PTCH2SRV_P | 0.400000005960465 | 1 | unknown | | false |  | Not Yet
PTCH2SRV_D | 0.0199999995529652 | 1 | unknown | | false |  | Not Yet
PTCH2SRV_I | 0.0399999991059303 | 1 | unknown | | false |  | Not Yet
PTCH2SRV_RMAX_UP | 0 | 1 | unknown | | false |  | Not Yet
PTCH2SRV_RMAX_DN | 0 | 1 | unknown | | false |  | Not Yet
PTCH2SRV_RLL | 1 | 1 | unknown | | false |  | Not Yet
PTCH2SRV_IMAX | 3000 | 1 | unknown | | false |  | Not Yet
PTCH2SRV_FF | 0 | 1 | unknown | | false |  | Not Yet
YAW2SRV_SLIP | 0 | 1 | unknown | | false |  | Not Yet
YAW2SRV_INT | 0 | 1 | unknown | | false |  | Not Yet
YAW2SRV_DAMP | 0 | 1 | unknown | | false |  | Not Yet
YAW2SRV_RLL | 1 | 1 | unknown | | false |  | Not Yet
YAW2SRV_IMAX | 1500 | 1 | unknown | | false |  | Not Yet
STEER2SRV_TCONST | 0.75 | 1 | unknown | | false |  | Not Yet
STEER2SRV_P | 1.79999995231628 | 1 | unknown | | false |  | Not Yet
STEER2SRV_I | 0.200000002980232 | 1 | unknown | | false |  | Not Yet
STEER2SRV_D | 0.00499999988824129 | 1 | unknown | | false |  | Not Yet
STEER2SRV_IMAX | 1500 | 1 | unknown | | false |  | Not Yet
STEER2SRV_MINSPD | 1 | 1 | unknown | | false |  | Not Yet
STEER2SRV_FF | 0 | 1 | unknown | | false |  | Not Yet
COMPASS_OFS_X | 0 | 1 | unknown | | false |  | Not Yet
COMPASS_OFS_Y | 0 | 1 | unknown | | false |  | Not Yet
COMPASS_OFS_Z | 0 | 1 | unknown | | false |  | Not Yet
COMPASS_LEARN | 1 | 1 | unknown | | false |  | Not Yet
COMPASS_USE | 1 | 1 | unknown | | false |  | Not Yet
COMPASS_AUTODEC | 1 | 1 | unknown | | false |  | Not Yet
COMPASS_MOTCT | 0 | 1 | unknown | | false |  | Not Yet
COMPASS_MOT_X | 0 | 1 | unknown | | false |  | Not Yet
COMPASS_MOT_Y | 0 | 1 | unknown | | false |  | Not Yet
COMPASS_MOT_Z | 0 | 1 | unknown | | false |  | Not Yet
COMPASS_ORIENT | 0 | 1 | unknown | | false |  | Not Yet
COMPASS_EXTERNAL | 0 | 1 | unknown | | false |  | Not Yet
SCHED_DEBUG | 0 | 1 | unknown | | false |  | Not Yet
RCMAP_ROLL | 1 | 1 | unknown | | false |  | Not Yet
RCMAP_PITCH | 2 | 1 | unknown | | false |  | Not Yet
RCMAP_THROTTLE | 3 | 1 | unknown | | false |  | Not Yet
RCMAP_YAW | 4 | 1 | unknown | | false |  | Not Yet
SR0_RAW_SENS | 1 | 1 | unknown | | false |  | Not Yet
SR0_EXT_STAT | 1 | 1 | unknown | | false |  | Not Yet
SR0_RC_CHAN | 1 | 1 | unknown | | false |  | Not Yet
SR0_RAW_CTRL | 1 | 1 | unknown | | false |  | Not Yet
SR0_POSITION | 1 | 1 | unknown | | false |  | Not Yet
SR0_EXTRA1 | 10 | 1 | unknown | | false |  | Not Yet
SR0_EXTRA2 | 10 | 1 | unknown | | false |  | Not Yet
SR0_EXTRA3 | 1 | 1 | unknown | | false |  | Not Yet
SR0_PARAMS | 10 | 1 | unknown | | false |  | Not Yet
SR1_RAW_SENS | 1 | 1 | unknown | | false |  | Not Yet
SR1_EXT_STAT | 1 | 1 | unknown | | false |  | Not Yet
SR1_RC_CHAN | 1 | 1 | unknown | | false |  | Not Yet
SR1_RAW_CTRL | 1 | 1 | unknown | | false |  | Not Yet
SR1_POSITION | 1 | 1 | unknown | | false |  | Not Yet
SR1_EXTRA1 | 1 | 1 | unknown | | false |  | Not Yet
SR1_EXTRA2 | 1 | 1 | unknown | | false |  | Not Yet
SR1_EXTRA3 | 1 | 1 | unknown | | false |  | Not Yet
SR1_PARAMS | 10 | 1 | unknown | | false |  | Not Yet
INS_PRODUCT_ID | 88 | 1 | unknown | | false |  | Not Yet
INS_GYROFFS_X | 1.80917340912856E-4 | 1 | unknown | | false |  | Not Yet
INS_GYROFFS_Y | -0.0767195895314217 | 1 | unknown | | false |  | Not Yet
INS_GYROFFS_Z | 0.0333446636795998 | 1 | unknown | | false |  | Not Yet
INS_ACCSCAL_X | 1 | 1 | unknown | | false |  | Not Yet
INS_ACCSCAL_Y | 1 | 1 | unknown | | false |  | Not Yet
INS_ACCSCAL_Z | 1 | 1 | unknown | | false |  | Not Yet
INS_ACCOFFS_X | 0 | 1 | unknown | | false |  | Not Yet
INS_ACCOFFS_Y | 0 | 1 | unknown | | false |  | Not Yet
INS_ACCOFFS_Z | 0 | 1 | unknown | | false |  | Not Yet
INS_GYRO_FILTER | 20 | 1 | unknown | | false |  | Not Yet
INS_ACCEL_FILTER | 20 | 1 | unknown | | false |  | Not Yet
INS_USE | 1 | 1 | unknown | | false |  | Not Yet
AHRS_GPS_GAIN | 1 | 1 | unknown | | false |  | Not Yet
AHRS_GPS_USE | 1 | 1 | unknown | | false |  | Not Yet
AHRS_YAW_P | 0.200000002980232 | 1 | unknown | | false |  | Not Yet
AHRS_RP_P | 0.200000002980232 | 1 | unknown | | false |  | Not Yet
AHRS_WIND_MAX | 0 | 1 | unknown | | false |  | Not Yet
AHRS_TRIM_X | 0 | 1 | unknown | | false |  | Not Yet
AHRS_TRIM_Y | 0 | 1 | unknown | | false |  | Not Yet
AHRS_TRIM_Z | 0 | 1 | unknown | | false |  | Not Yet
AHRS_ORIENTATION | 0 | 1 | unknown | | false |  | Not Yet
AHRS_COMP_BETA | 0.100000001490116 | 1 | unknown | | false |  | Not Yet
AHRS_GPS_MINSATS | 6 | 1 | unknown | | false |  | Not Yet
ARSPD_ENABLE | 1 | 1 | unknown | | false |  | Not Yet
ARSPD_USE | 0 | 1 | unknown | | false |  | Not Yet
ARSPD_OFFSET | 860.682922363281 | 1 | unknown | | false |  | Not Yet
ARSPD_RATIO | 1.99360001087189 | 1 | unknown | | false |  | Not Yet
ARSPD_PIN | 0 | 1 | unknown | | false |  | Not Yet
ARSPD_AUTOCAL | 0 | 1 | unknown | | false |  | Not Yet
ARSPD_TUBE_ORDER | 2 | 1 | unknown | | false |  | Not Yet
ARSPD_SKIP_CAL | 0 | 1 | unknown | | false |  | Not Yet
NAVL1_PERIOD | 20 | 1 | unknown | | false |  | Not Yet
NAVL1_DAMPING | 0.75 | 1 | unknown | | false |  | Not Yet
TECS_CLMB_MAX | 5 | 1 | unknown | | false |  | Not Yet
TECS_SINK_MIN | 2 | 1 | unknown | | false |  | Not Yet
TECS_TIME_CONST | 5 | 1 | unknown | | false |  | Not Yet
TECS_THR_DAMP | 0.5 | 1 | unknown | | false |  | Not Yet
TECS_INTEG_GAIN | 0.100000001490116 | 1 | unknown | | false |  | Not Yet
TECS_VERT_ACC | 7 | 1 | unknown | | false |  | Not Yet
TECS_HGT_OMEGA | 3 | 1 | unknown | | false |  | Not Yet
TECS_SPD_OMEGA | 2 | 1 | unknown | | false |  | Not Yet
TECS_RLL2THR | 10 | 1 | unknown | | false |  | Not Yet
TECS_SPDWEIGHT | 1 | 1 | unknown | | false |  | Not Yet
TECS_PTCH_DAMP | 0 | 1 | unknown | | false |  | Not Yet
TECS_SINK_MAX | 5 | 1 | unknown | | false |  | Not Yet
TECS_LAND_ARSPD | -1 | 1 | unknown | | false |  | Not Yet
TECS_LAND_THR | -1 | 1 | unknown | | false |  | Not Yet
TECS_LAND_SPDWGT | 1 | 1 | unknown | | false |  | Not Yet
TECS_PITCH_MAX | 0 | 1 | unknown | | false |  | Not Yet
TECS_PITCH_MIN | 0 | 1 | unknown | | false |  | Not Yet
TECS_LAND_SINK | 0.25 | 1 | unknown | | false |  | Not Yet
TECS_LAND_TCONST | 2 | 1 | unknown | | false |  | Not Yet
TECS_LAND_DAMP | 0.5 | 1 | unknown | | false |  | Not Yet
TECS_LAND_PMAX | 10 | 1 | unknown | | false |  | Not Yet
BATT_MONITOR | 0 | 1 | unknown | | false |  | Not Yet
BATT_VOLT_PIN | 13 | 1 | unknown | | false |  | Not Yet
BATT_CURR_PIN | 12 | 1 | unknown | | false |  | Not Yet
BATT_VOLT_MULT | 10.1000003814697 | 1 | unknown | | false |  | Not Yet
BATT_AMP_PERVOLT | 17 | 1 | unknown | | false |  | Not Yet
BATT_AMP_OFFSET | 0 | 1 | unknown | | false |  | Not Yet
BATT_CAPACITY | 3300 | 1 | unknown | | false |  | Not Yet
BATT2_MONITOR | 0 | 1 | unknown | | false |  | Not Yet
BATT2_VOLT_PIN | 13 | 1 | unknown | | false |  | Not Yet
BATT2_CURR_PIN | 12 | 1 | unknown | | false |  | Not Yet
BATT2_VOLT_MULT | 10.1000003814697 | 1 | unknown | | false |  | Not Yet
BATT2_AMP_PERVOL | 17 | 1 | unknown | | false |  | Not Yet
BATT2_AMP_OFFSET | 0 | 1 | unknown | | false |  | Not Yet
BATT2_CAPACITY | 3300 | 1 | unknown | | false |  | Not Yet
BRD_SERIAL_NUM | 0 | 1 | unknown | | false |  | Not Yet
MIS_TOTAL | 0 | 1 | unknown | | false |  | The number of mission mission items that has been loaded by the ground station. Do not change this manually.
MIS_RESTART | 0 | 1 | list | 0:ResumeMission 1:RestartMission  | false |  | Not Yet
RALLY_TOTAL | 0 | 1 | unknown | | false |  | Not Yet
RALLY_LIMIT_KM | 5 | 1 | unknown | | false |  | Not Yet
RALLY_INCL_HOME | 0 | 1 | unknown | | false |  | Not Yet
COMPASS_DEC | 0.0694443434476852 | 1 | radian | -3.142 3.142 | false |  | An angle to compensate between the true north and magnetic north
