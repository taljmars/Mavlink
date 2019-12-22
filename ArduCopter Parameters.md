# ArduCopter Parameters

Name | Possible Value | Increment | Unit | Range | Read Only | Title | Description
--- | --- | --- | --- | --- | ---| --- | ---
ACRO_BAL_PITCH | 1 | 0.1 | unknown | 0 - 3 |  | Acro Balance Pitch | rate at which pitch angle returns to level in acro mode. A higher value causes the vehicle to return to level faster.
ACRO_BAL_ROLL | 1 | 0.1 | unknown | 0 - 3 |  | Acro Balance Roll | rate at which roll angle returns to level in acro mode. A higher value causes the vehicle to return to level faster.
ACRO_EXPO | 0.3 | 1 | unknown | |  |  | Coming soon
ACRO_RP_P | 4.5 | 1 |  | |  |  | Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode. Higher values mean faster rate of rotation.
ACRO_TRAINER | 2 | 1 | list | 0:Disabled<br/>1:Leveling<br/>2:LevelingAndLimited<br/> |  | Acro Trainer | 
ACRO_YAW_P | 4.5 | 1 |  | |  |  | Converts pilot yaw input into a desired rate of rotation in ACRO, Stabilize and SPORT modes. Higher values mean faster rate of rotation.
AHRS_COMP_BETA | 0.1 | 0.01 |  | 0.001 - 0.5 |  | AHRS Velocity Complementary Filter Beta Coefficient | This controls the time constant for the cross-over frequency used to fuse AHRS (MAV_PARAM_GROUP.ARDUCOPTER,airspeed and heading) and GPS data to estimate ground velocity. Time constant is 0.1/beta. A larger time constant will use GPS data less and a small time constant will use air data less.
AHRS_GPS_GAIN | 1 | 0.01 |  | 0.0 - 1.0 |  | AHRS GPS gain | This controls how how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.
AHRS_GPS_MINSATS | 6 | 1 |  | 0 - 10 |  | AHRS GPS Minimum satellites | Minimum number of satellites visible to use GPS for velocity based corrections attitude correction. This defaults to 6, which is about the point at which the velocity numbers from a GPS become too unreliable for accurate correction of the accelerometers.
AHRS_GPS_USE | 1 | 1 | list | 0:Disabled<br/>1:Enabled<br/> |  | AHRS use GPS for navigation | This controls whether to use dead-reckoning or GPS based navigation. If set to 0 then the GPS won’t be used for navigation, and only dead reckoning will be used. A value of zero should never be used for normal flight. Currently this affects only the DCM-based AHRS: the EKF uses GPS whenever it is available.
AHRS_ORIENTATION | 0 | 1 | list | 0:None<br/>1:Yaw45<br/>2:Yaw90<br/>3:Yaw135<br/>4:Yaw180<br/>5:Yaw225<br/>6:Yaw270<br/>7:Yaw315<br/>8:Roll180<br/>9:Roll180Yaw45<br/>10:Roll180Yaw90<br/>11:Roll180Yaw135<br/>12:Pitch180<br/>13:Roll180Yaw225<br/>14:Roll180Yaw270<br/>15:Roll180Yaw315<br/>16:Roll90<br/>17:Roll90Yaw45<br/>18:Roll90Yaw90<br/>19:Roll90Yaw135<br/>20:Roll270<br/>21:Roll270Yaw45<br/>22:Roll270Yaw90<br/>23:Roll270Yaw136<br/>24:Pitch90<br/>25:Pitch270<br/>26:Pitch180Yaw90<br/>27:Pitch180Yaw270<br/>28:Roll90Pitch90<br/>29:Roll180Pitch90<br/>30:Roll270Pitch90<br/>31:Roll90Pitch180<br/>32:Roll270Pitch180<br/>33:Roll90Pitch270<br/>34:Roll180Pitch270<br/>35:Roll270Pitch270<br/>36:Roll90Pitch180Yaw90<br/>37:Roll90Yaw270<br/> |  | Board Orientation | Overall board orientation relative to the standard orientation for the board type. This rotates the IMU and compass readings to allow the board to be oriented in your vehicle at any 90 or 45 degree angle. This option takes affect on next boot. After changing you will need to re-level your vehicle.
AHRS_RP_P | 0.1 | 0.01 | unknown | 0.1 - 0.4 |  | AHRS RP_P | This controls how fast the accelerometers correct the attitude
AHRS_TRIM_X | 4.882812E-4 | 0.01 | radian | -0.1745 - 0.1745 |  | AHRS Trim Roll | Compensates for the roll angle difference between the control board and the frame. Positive values make the vehicle roll right.
AHRS_TRIM_Y | 0.02518386 | 0.01 | radian | -0.1745 - 0.1745 |  | AHRS Trim Pitch | Compensates for the pitch angle difference between the control board and the frame. Positive values make the vehicle pitch up/back.
AHRS_TRIM_Z | 0 | 0.01 | radian | -0.1745 - 0.1745 |  | AHRS Trim Yaw | Not Used
AHRS_WIND_MAX | 0 | 1 | m/s | 0 - 127 |  | Maximum wind | This sets the maximum allowable difference between ground speed and airspeed. This allows the plane to cope with a failing airspeed sensor. A value of zero means to use the airspeed as is.
AHRS_YAW_P | 0.1 | 1 | unknown | 0.1 - 0.4 |  |  | This controls the weight the compass or GPS has on the heading. A higher value means the heading will track the yaw source (MAV_PARAM_GROUP.ARDUCOPTER,GPS or compass) more rapidly.
ANGLE_MAX | 4500 | 1 | centidegree | 1000 - 8000 |  | Angle Max | Maximum lean angle in all flight modes
ARMING_CHECK | 94 | 1 | list | 0:Disabled<br/>-17:SkipINS<br/>-33:SkipParams/Rangefinder<br/>-65:SkipRC<br/>1:Enabled<br/>-3:SkipBaro<br/>-5:SkipCompass<br/>-9:SkipGPS<br/>127:SkipVoltage<br/> |  |  | Allows enabling or disabling of pre-arming checks of receiver, accelerometer, barometer, compass and GPS
ATC_ACCEL_RP_MAX | 0 | 1000 | list | 0:Disabled<br/>18000:Slow<br/>36000:Medium<br/>54000:Fast<br/>9000:VerySlow<br/> |  | Acceleration Max for Roll/Pitch | Maximum acceleration in roll/pitch axis
ATC_ACCEL_Y_MAX | 0 | 1000 | list | 0:Disabled<br/>18000:Slow<br/>36000:Medium<br/>54000:Fast<br/>9000:VerySlow<br/> |  | Acceleration Max for Yaw | Maximum acceleration in yaw axis
ATC_RATE_FF_ENAB | 0 | 1 | list | 0:Disabled<br/>1:Enabled<br/> |  |  | Controls whether body-frame rate feedforward is enabled or disabled
ATC_RATE_RP_MAX | 18000 | 1 | list | 0:Disabled<br/>720:Medium<br/>360:Slow<br/>1080:Fast<br/> |  | Angular Velocity Max for Pitch/Roll | Maximum angular velocity in pitch/roll axis
ATC_RATE_Y_MAX | 9000 | 1 | list | 0:Disabled<br/>720:Medium<br/>360:Slow<br/>1080:Fast<br/> |  | Angular Velocity Max for Yaw | Maximum angular velocity in yaw axis
ATC_SLEW_YAW | 1000 | 1 | centidegrees/s | 500 - 18000 |  | Yaw target slew rate | Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
BAROGLTCH_ACCEL | 1500 | 1 | unknown | |  |  | Coming soon
BAROGLTCH_DIST | 500 | 1 | unknown | |  |  | Coming soon
BAROGLTCH_ENABLE | 1 | 1 | unknown | |  |  | Coming soon
BATT_AMP_OFFSET | 0 | 1 | volt | |  | AMP offset | Voltage offset at zero current on current sensor
BATT_AMP_PERVOLT | 17 | 1 | amper/volt | |  | Amps per volt | Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.
BATT_AMP_PERVLT | 17 | 1 | amper/volt | |  | Amps per volt | Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.
BATT_CAPACITY | 4000 | 1 | mAh | |  | Battery capacity | Capacity of the battery in mAh when full
BATT_CURR_PIN | 3 | 1 | list | -1:Disabled<br/>1:A1<br/>2:A2<br/>3:Pixhawk<br/>101:PX4<br/>12:A12<br/> |  |  | Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk powered from the PM connector it should be set to 3.
BATT_MONITOR | 4 | 1 | list | 0:Disabled<br/>3:Analog_Voltage_Only<br/>4:Analog_Voltage_and_Current<br/>5:SMBus<br/>6:Bebop<br/> |  | Battery monitoring | Controls enabling monitoring of the battery's voltage and current
BATT_VOLT_MULT | 10.1 | 1 | unknown | |  | Voltage Multiplier | Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (MAV_PARAM_GROUP.ARDUCOPTER,pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX4 using the PX4IO power supply this should be set to 1.
BATT_VOLT_PIN | 2 | 1 | list | -1:Disabled<br/>0:A0<br/>1:A1<br/>2:Pixhawk<br/>100:PX4<br/>13:A13<br/> |  | Battery Voltage sensing pin | Setting this to 0 ~ 13 will enable battery voltage sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
BATT_VOLT2_MULT | 1 | 1 | unknown | |  | Voltage Multiplier | Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery’s voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1
BATT_VOLT2_PIN | -1 | 1 | unknown | |  | Battery Voltage sensing pin | Sets the analog input pin that should be used for voltage monitoring.
CAM_DURATION | 10 | 1 | decisecond | 0 - 50 |  | Duration that shutter is held open | How long the shutter will be held open in 10ths of a second (i.e. enter 10 for 1second, 50 for 5seconds)
CAM_SERVO_OFF | 1100 | 1 | PWM/ms | 1000 - 2000 |  | Servo OFF PWM value | PWM value to move servo to when shutter is deactivated
CAM_SERVO_ON | 1300 | 1 | PWM/ms | 1000 - 2000 |  | Servo ON PWM value | PWM value to move servo to when shutter is activated
CAM_TRIGG_DIST | 0 | 1 | meter | 0 - 1000 |  | Camera trigger distance | Distance in meters between camera triggers. If this value is non-zero then the camera will trigger whenever the GPS position changes by this number of meters regardless of what mode the APM is in. Note that this parameter can also be set in an auto mission using the DO_SET_CAM_TRIGG_DIST command, allowing you to enable/disable the triggering of the camera during the flight.
CAM_TRIGG_TYPE | 0 | 1 | list | 0:Servo<br/>1:Relay<br/> |  | Camera shutter (trigger) type | how to trigger the camera to take a picture
CH7_OPT | 17 | 1 | list | 0:DoNothing<br/>2:Flip<br/>3:SimpleMode<br/>4:RTL<br/>5:SaveTrim<br/>7:SaveWP<br/>9:CameraTrigger<br/>10:RangeFinder<br/>11:Fence<br/>13:SuperSimpleMode<br/>14:AcroTrainer<br/>15:Sprayer<br/>16:Auto<br/>17:AutoTune<br/>18:Land<br/>19:EPM<br/>21:ParachuteEnable<br/>22:ParachuteRelease<br/>23:Parachute3pos<br/>24:AutoMissionReset<br/>25:AttConFeedForward<br/>26:AttConAccelLimits<br/>27:RetractMount<br/>28:Relay-On/Off<br/>29:Landing-Gear<br/>30:Lost-Copter-Sound<br/>31:Motor-Emergency-Stop<br/>32:MotorInterlock<br/>33:Brake<br/>34:Relay2-On/Off<br/>35:Relay3-On/Off<br/>36:Relay4-On/Off<br/>37:Throw<br/>38:Avoidance<br/> |  | Channel 7 option | Select which function is performed when CH7 is above 1800 pwm
CH8_OPT | 0 | 1 | list | 0:DoNothing<br/>2:Flip<br/>3:SimpleMode<br/>4:RTL<br/>5:SaveTrim<br/>7:SaveWP<br/>9:CameraTrigger<br/>10:RangeFinder<br/>11:Fence<br/>13:SuperSimpleMode<br/>14:AcroTrainer<br/>15:Sprayer<br/>16:Auto<br/>17:AutoTune<br/>18:Land<br/>19:EPM<br/>21:ParachuteEnable<br/>22:ParachuteRelease<br/>23:Parachute3pos<br/>24:AutoMissionReset<br/>25:AttConFeedForward<br/>26:AttConAccelLimitsRetractMount<br/>28:Relay-On/Off<br/>29:Landing-Gear<br/>30:Lost-Copter-Sound<br/>31:Motor-Emergency-Stop<br/>32:MotorInterlock<br/>33:Brake<br/>34:Relay2-On/Off<br/>35:Relay3-On/Off<br/>36:Relay4-On/Off<br/>37:Throw<br/>38:Avoidance<br/> |  | Channel 8 option | Select which function is performed when CH8 is above 1800 pwm
CIRCLE_RADIUS | 500 | 1 | cm | 0 - 10000 |  |  | Defines the radius of the circle the vehicle will fly when in Circle flight mode
CIRCLE_RATE | 20 | 1 | deg/s | -90 - 90 |  |  | Circle mode's turn rate in deg/sec. Positive to turn clockwise, negative for counter clockwise
COMPASS_AUTODEC | 1 | 1 | list | 0:Disabled<br/>1:Enabled<br/> |  | Auto Declination | Enable or disable the automatic calculation of the declination based on gps location
COMPASS_DEC | 0.06941485 | 0.01 | radian | -3.142 - 3.142 |  | Compass declination | An angle to compensate between the true north and magnetic north
COMPASS_EXTERNAL | 0 | 1 | list | 0:Internal<br/>1:External<br/>2:ForcedExternal<br/> |  |  | Configure compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
COMPASS_LEARN | 0 | 1 | list | 0:Disabled<br/>1:Internal-Learning<br/>2:EKF-Learning<br/> |  | Learn compass offsets automatically | Enable or disable the automatic learning of compass offsets. You can enable learning either using a compass-only method that is suitable only for fixed wing aircraft or using the offsets learnt by the active EKF state estimator. If this option is enabled then the learnt offsets are saved when you disarm the vehicle.
COMPASS_MOT_X | 0 | 1 | milligauss/amper | -1000 - 1000 |  | Motor interference compensation for body frame X axis | Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference
COMPASS_MOT_Y | 0 | 1 | milligauss/amper | -1000 - 1000 |  | Motor interference compensation for body frame Y axis | Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference
COMPASS_MOT_Z | 0 | 1 | milligauss/amper | -1000 - 1000 |  | Motor interference compensation for body frame Z axis | Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference
COMPASS_MOTCT | 0 | 1 | list | 0:Disabled<br/>1:UseThrottle<br/>2:UseCurrent<br/> |  | Motor interference compensation type | Set motor interference compensation type to disabled, throttle or current. Do not change manually.
COMPASS_OFS_X | -98 | 1 | milligauss | -400 - 400 |  | Compass offsets in milligauss on the X axis | Offset to be added to the compass x-axis values to compensate for metal in the frame
COMPASS_OFS_Y | 29 | 1 | milligauss | -400 - 400 |  | Compass offsets in milligauss on the Y axis | Offset to be added to the compass y-axis values to compensate for metal in the frame
COMPASS_OFS_Z | -18 | 1 | milligauss | -400 - 400 |  | Compass offsets in milligauss on the X axis | Offset to be added to the compass z-axis values to compensate for metal in the frame
COMPASS_ORIENT | 0 | 1 | list | 0:None<br/>1:Yaw45<br/>2:Yaw90<br/>3:Yaw135<br/>4:Yaw180<br/>5:Yaw225<br/>6:Yaw270<br/>7:Yaw315<br/>8:Roll180<br/>9:Roll180Yaw45<br/>10:Roll180Yaw90<br/>11:Roll180Yaw135<br/>12:Pitch180<br/>13:Roll180Yaw225<br/>14:Roll180Yaw270<br/>15:Roll180Yaw315<br/>16:Roll90<br/>17:Roll90Yaw45<br/>18:Roll90Yaw90<br/>19:Roll90Yaw135<br/>20:Roll270<br/>21:Roll270Yaw45<br/>22:Roll270Yaw90<br/>23:Roll270Yaw136<br/>24:Pitch90<br/>25:Pitch270<br/>26:Pitch180Yaw90<br/>27:Pitch180Yaw270<br/>28:Roll90Pitch90<br/>29:Roll180Pitch90<br/>30:Roll270Pitch90<br/>31:Roll90Pitch180<br/>32:Roll270Pitch180<br/>33:Roll90Pitch270<br/>34:Roll180Pitch270<br/>35:Roll270Pitch270<br/>36:Roll90Pitch180Yaw90<br/>37:Roll90Yaw270<br/>38:Yaw293Pitch68Roll90<br/> |  |  | The orientation of the compass relative to the autopilot board. This will default to the right value for each board type, but can be changed if you have an external compass. See the documentation for your external compass for the right value. The correct orientation should give the X axis forward, the Y axis to the right and the Z axis down. So if your aircraft is pointing west it should show a positive value for the Y axis, and a value close to zero for the X axis. On a PX4 or Pixhawk with an external compass the correct value is zero if the compass is correctly oriented. NOTE: This orientation is combined with any AHRS_ORIENTATION setting.
COMPASS_USE | 1 | 1 | list | 0:Disabled<br/>1:Enabled<br/> |  | Use compass for yaw | Enable or disable the use of the compass (MAV_PARAM_GROUP.ARDUCOPTER,instead of the GPS) for determining heading
DCM_CHECK_THRESH | 0.8 | 1 | unknown | |  |  | Coming soon
EKF_CHECK_THRESH | 0.8 | 1 | unknown | |  |  | Coming soon
ESC | 0 | 1 | unknown | |  |  | Coming soon
FENCE_ACTION | 1 | 1 | list | 0:ReportOnly<br/>1:RTL-or-Land<br/> |  |  | What action should be taken when fence is breached
FENCE_ALT_MAX | 100 | 1 | meter | 10 - 1000 |  | Fence Maximum Altitude | Maximum altitude allowed before geofence triggers
FENCE_ENABLE | 1 | 1 | list | 0:Disabled<br/>1:Enabled<br/> |  |  | Allows you to enable (1) or disable (0) the fence functionality
FENCE_MARGIN | 2 | 1 | meter | 1 - 10 |  | Fence Margin | Distance that autopilot's should maintain from the fence to avoid a breach
FENCE_RADIUS | 300 | 1 | meter | 30 - 10000 |  |  | Circle fence radius which when breached will cause an RTL
FENCE_TYPE | 1 | 1 | list | 0:None<br/>1:Altitude<br/>2:Circle<br/>3:AltitudeAndCircle<br/>4:Polygon<br/>5:AltitudeAndPolygon<br/>6:CircleAndPolygon<br/>7:All<br/> |  |  | Enabled fence types held as bitmask
FLOW_ENABLE | 0 | 1 | list | 0:Disabled<br/>1:Enabled<br/> |  |  | Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will disable optical flow
FLTMODE1 | 0 | 1 | list | 0:Stabilize<br/>1:Acro<br/>2:AltHold<br/>3:Auto<br/>4:Guided<br/>5:Loiter<br/>6:RTL<br/>7:Circle<br/>9:Land<br/>11:Drift<br/>13:Sport<br/>14:Flip<br/>15:AutoTune<br/>16:PosHold<br/>17:Brake<br/>18:Throw<br/>19:Avoid_ADSB<br/>20:Guided_NoGPS<br/> |  |  | Flight mode when Channel 5 pwm is <= 1230
FLTMODE2 | 0 | 1 | list | 0:Stabilize<br/>1:Acro<br/>2:AltHold<br/>3:Auto<br/>4:Guided<br/>5:Loiter<br/>6:RTL<br/>7:Circle<br/>9:Land<br/>11:Drift<br/>13:Sport<br/>14:Flip<br/>15:AutoTune<br/>16:PosHold<br/>17:Brake<br/>18:Throw<br/>19:Avoid_ADSB<br/>20:Guided_NoGPS<br/> |  |  | Flight mode when Channel 5 pwm is >1230, <= 1360
FLTMODE3 | 16 | 1 | list | 0:Stabilize<br/>1:Acro<br/>2:AltHold<br/>3:Auto<br/>4:Guided<br/>5:Loiter<br/>6:RTL<br/>7:Circle<br/>9:Land<br/>11:Drift<br/>13:Sport<br/>14:Flip<br/>15:AutoTune<br/>16:PosHold<br/>17:Brake<br/>18:Throw<br/>19:Avoid_ADSB<br/>20:Guided_NoGPS<br/> |  |  | Flight mode when Channel 5 pwm is >1360, <= 1490
FLTMODE4 | 16 | 1 | list | 0:Stabilize<br/>1:Acro<br/>2:AltHold<br/>3:Auto<br/>4:Guided<br/>5:Loiter<br/>6:RTL<br/>7:Circle<br/>9:Land<br/>11:Drift<br/>13:Sport<br/>14:Flip<br/>15:AutoTune<br/>16:PosHold<br/>17:Brake<br/>18:Throw<br/>19:Avoid_ADSB<br/>20:Guided_NoGPS<br/> |  |  | Flight mode when Channel 5 pwm is >1490, <= 1620
FLTMODE5 | 6 | 1 | list | 0:Stabilize<br/>1:Acro<br/>2:AltHold<br/>3:Auto<br/>4:Guided<br/>5:Loiter<br/>6:RTL<br/>7:Circle<br/>9:Land<br/>11:Drift<br/>13:Sport<br/>14:Flip<br/>15:AutoTune<br/>16:PosHold<br/>17:Brake<br/>18:Throw<br/>19:Avoid_ADSB<br/>20:Guided_NoGPS<br/> |  |  | Flight mode when Channel 5 pwm is >1620, <= 1749
FLTMODE6 | 6 | 1 | list | 0:Stabilize<br/>1:Acro<br/>2:AltHold<br/>3:Auto<br/>4:Guided<br/>5:Loiter<br/>6:RTL<br/>7:Circle<br/>9:Land<br/>11:Drift<br/>13:Sport<br/>14:Flip<br/>15:AutoTune<br/>16:PosHold<br/>17:Brake<br/>18:Throw<br/>19:Avoid_ADSB<br/>20:Guided_NoGPS<br/> |  |  | Flight mode when Channel 5 pwm is >=1750
FRAME | 1 | 1 | list | 0:Plus<br/>1:X<br/>2:V<br/>3:H<br/>4:V-Tail<br/>5:A-Tail<br/>10:Y6B-New)<br/> |  |  | Controls motor mixing for multicopters. Not used for Tri or Traditional Helicopters.
FS_BATT_ENABLE | 0 | 1 | list | 0:Disabled<br/>1:Land<br/>2:RTL<br/> |  |  | Controls whether failsafe will be invoked when battery voltage or current runs low
FS_BATT_MAH | 0 | 1 | mAh | |  |  | Battery capacity remaining to trigger failsafe. Set to 0 to disable battery remaining failsafe. If the battery remaining drops below this level then the copter will RTL
FS_BATT_VOLTAGE | 14 | 1 | volt | |  |  | Battery voltage to trigger failsafe. Set to 0 to disable battery voltage failsafe. If the battery voltage drops below this voltage then the copter will RTL
FS_GCS_ENABLE | 0 | 1 | list | 0:Disabled<br/>1:Enabled_always_RTL<br/>2:Enabled_Continue_with_Mission_in_Auto_Mode<br/> |  |  | Controls whether failsafe will be invoked (MAV_PARAM_GROUP.ARDUCOPTER,and what action to take) when connection with Ground station is lost for at least 5 seconds. NB. The GCS Failsafe is only active when RC_OVERRIDE is being used to control the vehicle.
FS_GPS_ENABLE | 1 | 1 | unknown | |  |  | Coming soon
FS_THR_ENABLE | 1 | 1 | list | 0:Disabled<br/>1:Enabled_Always_RTL<br/>2:Enabled_Continue_with_Mission_in_Auto_Mode<br/>3:Enabled_Always_LAND<br/> |  |  | The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
FS_THR_VALUE | 975 | 1 | PWM | 925 - 1100 |  |  | The PWM level on channel 3 below which throttle failsafe triggers
GND_ABS_PRESS | 50362.25 | 1 | pascal | | V |  | calibrated ground pressure in PASCAL
GND_ALT_OFFSET | 0 | 1 | meter | |  |  | altitude offset in meters added to barometric altitude. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer. The value is added to the barometric altitude read by the aircraft. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed.
GND_TEMP | 29.32966 | 1 | degrees-celsius | |  |  | calibrated ground temperature in degrees Celsius
GPS_HDOP_GOOD | 230 | 1 | unknown | 100 - 900 |  |  | GPS Hdop value at or below this value represent a good position. Used for pre-arm checks
GPS_NAVFILTER | 8 | 1 | list | 0:Portable<br/>2:Stationary<br/>3:Pedestrian<br/>4:Automotive<br/>5:Sea<br/>6:Airborne1G<br/>7:Airborne2G<br/>8:Airborne4G<br/> |  |  | Navigation filter engine setting
GPS_TYPE | 1 | 1 | list | 0:None<br/>1:AUTO<br/>2:uBlox<br/>3:MTK<br/>4:MTK19<br/>5:NMEA<br/>6:SiRF<br/>7:HIL<br/>8:SwiftNav<br/>9:PX4-UAVCAN<br/>10:SBF<br/>11:GSOF<br/>12:QURT<br/>13:ERB<br/>14:MAV<br/>15:NOVA<br/> |  |  | GPS type
GPSGLITCH_ACCEL | 1000 | 1 | unknown | |  |  | Coming soon
GPSGLITCH_ENABLE | 1 | 1 | unknown | |  |  | Coming soon
GPSGLITCH_RADIUS | 200 | 1 | unknown | |  |  | Coming soon
HLD_LAT_P | 1 | 1 | unknown | |  |  | Coming soon
INAV_TC_XY | 2.5 | 1 | unknown | |  |  | Coming soon
INAV_TC_Z | 5 | 1 | unknown | |  |  | Coming soon
INS_ACCOFFS_X | 0.02704384 | 1 | m/s/s | -3.5 - 3.5 |  | Accelerometer offsets of X axis | This is setup using the acceleration calibration or level operations
INS_ACCOFFS_Y | 0.07232188 | 1 | m/s/s | -3.5 - 3.5 |  | Accelerometer offsets of Y axis | This is setup using the acceleration calibration or level operations
INS_ACCOFFS_Z | -0.2916607 | 1 | m/s/s | -3.5 - 3.5 |  | Accelerometer offsets of Z axis | This is setup using the acceleration calibration or level operations
INS_ACCSCAL_X | 0.9998162 | 1 | unknown | 0.8 - 1.2 |  | Accelerometer scaling of X axis | Calculated during acceleration calibration routine
INS_ACCSCAL_Y | 0.9920868 | 1 | unknown | 0.8 - 1.2 |  | Accelerometer scaling of Y axis | Calculated during acceleration calibration routine
INS_ACCSCAL_Z | 1.0001 | 1 | unknown | 0.8 - 1.2 |  | Accelerometer scaling of Z axis | Calculated during acceleration calibration routine
INS_GYROFFS_X | -0.003293759 | 1 | rad/s | |  | Gyro sensor offsets of X axis | This is setup on each boot during gyro calibrations
INS_GYROFFS_Y | -0.07851811 | 1 | rad/s | |  | Gyro sensor offsets of Y axis | This is setup on each boot during gyro calibrations
INS_GYROFFS_Z | 0.02421099 | 1 | rad/s | |  | Gyro sensor offsets of Z axis | This is setup on each boot during gyro calibrations
INS_MPU6K_FILTER | 0 | 1 | unknown | |  |  | Coming soon
INS_PRODUCT_ID | 0 | 1 | list | 0:Unknown<br/>256:unused<br/>1:unused<br/>257:Linux<br/>2:unused<br/>3:SITL<br/>4:PX4v1<br/>5:PX4v2<br/>88:unused<br/> |  |  | Which type of IMU is installed (MAV_PARAM_GROUP.ARDUCOPTER,read-only).
LAND_REPOSITION | 1 | 1 | list | 0:NoRepositioning<br/>1:Repositioning<br/> |  |  | Enables user input during LAND mode, the landing phase of RTL, and auto mode landings.
LAND_SPEED | 50 | 1 | cm/s | 30 - 200 |  |  | The descent speed for the final stage of landing in CENTIMETER/s
LOG_BITMASK | 26622 | 1 | list | 0:Disabled<br/>-6146:NearlyAll-AC315<br/>655358:All+FullIMU<br/>397310:All+FastIMU+PID<br/>393214:All+FastIMU<br/>262142:All+MotBatt<br/>830:Default<br/>894:Default+RCIN<br/>958:Default+IMU<br/>1854:Default+Motors<br/>45054:NearlyAll<br/>131071:All+FastATT<br/> |  |  | 4 byte bitmap of log types to enable, Bitmask: 0:ATTITUDE_FAST,1:ATTITUDE_MED,2:GPS,3:PM,4:CTUN,5:NTUN,6:RCIN,7:IMU,8:CMD,9:CURRENT,10:RCOUT,11:OPTFLOW,12:PID,13:COMPASS,14:INAV,15:CAMERA,17:MOTBATT,18:IMU_FAST,19:IMU_RAW
LOITER_LAT_D | 0 | 1 | unknown | |  |  | Coming soon
LOITER_LAT_I | 0.5 | 1 | unknown | |  |  | Coming soon
LOITER_LAT_IMAX | 1000 | 1 | unknown | |  |  | Coming soon
LOITER_LAT_P | 1 | 1 | unknown | |  |  | Coming soon
LOITER_LON_D | 0 | 1 | unknown | |  |  | Coming soon
LOITER_LON_I | 0.5 | 1 | unknown | |  |  | Coming soon
LOITER_LON_IMAX | 1000 | 1 | unknown | |  |  | Coming soon
LOITER_LON_P | 1 | 1 | unknown | |  |  | Coming soon
MAG_ENABLE | 1 | 1 | list | 0:Disabled<br/>1:Enabled<br/> |  |  | Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
MIS_RESTART | 0 | 1 | list | 0:ResumeMission<br/>1:RestartMission<br/> |  | Mission Restart when entering Auto mode | Controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)
MIS_TOTAL | 6 | 1 |  | 0 - 32766 | V | Total mission commands | The number of mission mission items that has been loaded by the ground station. Do not change this manually.
MNT_ANGMAX_PAN | 4500 | 1 | centidegree | -18000 - 17999 |  |  | Maximum physical pan (yaw) angular position of the mount
MNT_ANGMAX_ROL | 4500 | 1 | centidegree | -18000 - 17999 |  |  | Maximum physical roll angular position of the mount
MNT_ANGMAX_TIL | 0 | 1 | centidegree | -18000 - 17999 |  |  | Maximum physical tilt (pitch) angular position of the mount
MNT_ANGMIN_PAN | -4500 | 1 | centidegree | -18000 - 17999 |  |  | Minimum physical pan (yaw) angular position of mount.
MNT_ANGMIN_ROL | -4500 | 1 | centidegree | -18000 - 17999 |  |  | Minimum physical roll angular position of mount.
MNT_ANGMIN_TIL | -9000 | 1 | centidegree | -18000 - 17999 |  |  | Minimum physical tilt (pitch) angular position of mount.
MNT_CONTROL_X | 0 | 1 | unknown | |  |  | Coming soon
MNT_CONTROL_Y | 0 | 1 | unknown | |  |  | Coming soon
MNT_CONTROL_Z | 0 | 1 | unknown | |  |  | Coming soon
MNT_JSTICK_SPD | 0 | 1 | unknown | 0 - 100 |  |  | 0 for position control, small for low speeds, 100 for max speed. A good general value is 10 which gives a movement speed of 3 degrees per second.
MNT_MODE | 3 | 1 | unknown | |  |  | Coming soon
MNT_NEUTRAL_X | 0 | 1 | degree | -180.0 - 179.99 |  |  | Mount roll angle when in neutral position
MNT_NEUTRAL_Y | 0 | 1 | degree | -180.0 - 179.99 |  |  | Mount tilt/pitch angle when in neutral position
MNT_NEUTRAL_Z | 0 | 1 | degree | -180.0 - 179.99 |  |  | Mount pan/yaw angle when in neutral position
MNT_RC_IN_PAN | 0 | 1 | list | 0:Disabled<br/>5:RC5<br/>6:RC6<br/>7:RC7<br/>8:RC8<br/>9:RC9<br/>10:RC10<br/>11:RC11<br/>12:RC12<br/> |  |  | 0 for none, any other for the RC channel to be used to control pan (MAV_PARAM_GROUP.ARDUCOPTER,yaw) movements
MNT_RC_IN_ROLL | 0 | 1 | list | 0:Disabled<br/>5:RC5<br/>6:RC6<br/>7:RC7<br/>8:RC8<br/>9:RC9<br/>10:RC10<br/>11:RC11<br/>12:RC12<br/> |  |  | 0 for none, any other for the RC channel to be used to control roll movements
MNT_RC_IN_TILT | 6 | 1 | list | 0:Disabled<br/>5:RC5<br/>6:RC6<br/>7:RC7<br/>8:RC8<br/>9:RC9<br/>10:RC10<br/>11:RC11<br/>12:RC12<br/> |  |  | 0 for none, any other for the RC channel to be used to control tilt (MAV_PARAM_GROUP.ARDUCOPTER,pitch) movements
MNT_RETRACT_X | 0 | 1 | degree | -180.0 - 179.99 |  |  | Mount roll angle when in retracted position
MNT_RETRACT_Y | 0 | 1 | degree | -180.0 - 179.99 |  |  | Mount tilt/pitch angle when in retracted position
MNT_RETRACT_Z | 0 | 1 | degree | -180.0 - 179.99 |  |  | Mount yaw/pan angle when in retracted position
MNT_STAB_PAN | 0 | 1 | list | 0:Disabled<br/>1:Enabled<br/> |  |  | enable pan/yaw stabilisation relative to Earth
MNT_STAB_ROLL | 0 | 1 | list | 0:Disabled<br/>1:Enabled<br/> |  |  | enable roll stabilisation relative to Earth
MNT_STAB_TILT | 0 | 1 | list | 0:Disabled<br/>1:Enabled<br/> |  |  | enable tilt/pitch stabilisation relative to Earth
MOT_SPIN_ARMED | 90 | 1 | unknown | |  |  | Coming soon
MOT_TCRV_ENABLE | 1 | 1 | unknown | |  |  | Coming soon
MOT_TCRV_MAXPCT | 93 | 1 | unknown | |  |  | Coming soon
MOT_TCRV_MIDPCT | 52 | 1 | unknown | |  |  | Coming soon
OF_PIT_D | 0.12 | 1 | unknown | |  |  | Coming soon
OF_PIT_I | 0.5 | 1 | unknown | |  |  | Coming soon
OF_PIT_IMAX | 100 | 1 | unknown | |  |  | Coming soon
OF_PIT_P | 2.5 | 1 | unknown | |  |  | Coming soon
OF_RLL_D | 0.12 | 1 | unknown | |  |  | Coming soon
OF_RLL_I | 0.5 | 1 | unknown | |  |  | Coming soon
OF_RLL_IMAX | 100 | 1 | unknown | |  |  | Coming soon
OF_RLL_P | 2.5 | 1 | unknown | |  |  | Coming soon
PHLD_BRAKE_ANGLE | 3000 | 1 | centidegree | 2000 - 4500 |  |  | PosHold flight mode's max lean angle during braking in centi-degrees
PHLD_BRAKE_RATE | 8 | 1 | deg/s | 4 - 12 |  | PosHold braking rate | PosHold flight mode's rotation rate during braking in deg/sec
PILOT_ACCEL_Z | 250 | 1 | cm/s/s | 50 - 500 |  |  | The vertical acceleration used when pilot is controlling the altitude
PILOT_VELZ_MAX | 250 | 1 | cm/s | 50 - 500 |  |  | The maximum vertical velocity the pilot may request in CENTIMETER/s
POSCON_THR_HOVER | 724 | 1 | unknown | |  |  | Coming soon
RATE_PIT_D | 0.0055 | 1 | unknown | |  |  | Coming soon
RATE_PIT_I | 0.07999999 | 1 | unknown | |  |  | Coming soon
RATE_PIT_IMAX | 1000 | 1 | unknown | |  |  | Coming soon
RATE_PIT_P | 0.07999999 | 1 | unknown | |  |  | Coming soon
RATE_RLL_D | 0.003 | 1 | unknown | |  |  | Coming soon
RATE_RLL_I | 0.08499999 | 1 | unknown | |  |  | Coming soon
RATE_RLL_IMAX | 1000 | 1 | unknown | |  |  | Coming soon
RATE_RLL_P | 0.08499999 | 1 | unknown | |  |  | Coming soon
RATE_YAW_D | 0.003 | 1 | unknown | |  |  | Coming soon
RATE_YAW_I | 0.02 | 1 | unknown | |  |  | Coming soon
RATE_YAW_IMAX | 1000 | 1 | unknown | |  |  | Coming soon
RATE_YAW_P | 0.17 | 1 | unknown | |  |  | Coming soon
RC_FEEL_RP | 100 | 1 | list | 0:Standard<br/>50:Medium<br/>100:VeryCrisp<br/>1000:VerySoft<br/>25:Soft<br/>75:Crisp<br/> |  |  | RC feel for roll/pitch which controls vehicle response to user input with 0 being extremely soft and 100 being crisp
RC_SPEED | 490 | 1 | Hz | 50 - 490 |  |  | This is the speed in Hertz that your ESCs will receive updates
RC1_DZ | 30 | 1 | PWM | 0 - 200 |  | RC dead-zone | dead zone around trim or bottom
RC1_MAX | 1976 | 1 | PWM | 800 - 2200 |  | RC max PWM | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC1_MIN | 998 | 1 | PWM | 800 - 2200 |  | RC min PWM | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC1_REV | 1 | 1 | list | -1:Reversed<br/>1:Normal<br/> |  | RC reversed | Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
RC1_TRIM | 1483 | 1 | PWM | 800 - 2200 |  |  | RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC10_DZ | 0 | 1 | PWM | 0 - 200 |  | RC dead-zone | dead zone around trim or bottom
RC10_FUNCTION | 0 | 1 | list | 0:Disabled<br/>1:RCPassThru<br/>2:Flap<br/>3:Flap_auto<br/>4:Aileron<br/>6:mount_pan<br/>7:mount_tilt<br/>8:mount_roll<br/>9:mount_open<br/>10:camera_trigger<br/>11:release<br/>12:mount2_pan<br/>13:mount2_tilt<br/>14:mount2_roll<br/>15:mount2_open<br/>16:DifferentialSpoiler1<br/>17:DifferentialSpoiler2<br/>18:AileronWithInput<br/>19:Elevator<br/>20:ElevatorWithInput<br/>21:Rudder<br/>24:Flaperon1<br/>25:Flaperon2<br/>26:GroundSteering<br/>27:Parachute<br/>28:EPM<br/>29:LandingGear<br/>30:EngineRunEnable<br/>31:HeliRSC<br/>32:HeliTailRSC<br/>33:Motor1<br/>34:Motor2<br/>35:Motor3<br/>36:Motor4<br/>37:Motor5<br/>38:Motor6<br/>39:Motor7<br/>40:Motor8<br/>51:RCIN1<br/>52:RCIN2<br/>53:RCIN3<br/>54:RCIN4<br/>55:RCIN5<br/>56:RCIN6<br/>57:RCIN7<br/>58:RCIN8<br/>59:RCIN9<br/>60:RCIN10<br/>61:RCIN11<br/>62:RCIN12<br/>63:RCIN13<br/>64:RCIN14<br/>65:RCIN15<br/>66:RCIN16<br/>67:Ignition<br/>68:Choke<br/>69:Starter<br/>70:Throttle<br/> |  |  | Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
RC10_MAX | 1900 | 1 | PWM | 800 - 2200 |  | RC max PWM | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC10_MIN | 1100 | 1 | PWM | 800 - 2200 |  | RC min PWM | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC10_REV | 1 | 1 | list | -1:Reversed<br/>1:Normal<br/> |  | RC reversed | Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
RC10_TRIM | 0 | 1 | PWM | 800 - 2200 |  |  | RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC11_DZ | 0 | 1 | PWM | 0 - 200 |  |  | dead zone around trim or bottom
RC11_FUNCTION | 0 | 1 | list | 0:Disabled<br/>1:RCPassThru<br/>2:Flap<br/>3:Flap_auto<br/>4:Aileron<br/>6:mount_pan<br/>7:mount_tilt<br/>8:mount_roll<br/>9:mount_open<br/>10:camera_trigger<br/>11:release<br/>12:mount2_pan<br/>13:mount2_tilt<br/>14:mount2_roll<br/>15:mount2_open<br/>16:DifferentialSpoiler1<br/>17:DifferentialSpoiler2<br/>18:AileronWithInput<br/>19:Elevator<br/>20:ElevatorWithInput<br/>21:Rudder<br/>24:Flaperon1<br/>25:Flaperon2<br/>26:GroundSteering<br/>27:Parachute<br/>28:EPM<br/>29:LandingGear<br/>30:EngineRunEnable<br/>31:HeliRSC<br/>32:HeliTailRSC<br/>33:Motor1<br/>34:Motor2<br/>35:Motor3<br/>36:Motor4<br/>37:Motor5<br/>38:Motor6<br/>39:Motor7<br/>40:Motor8<br/>51:RCIN1<br/>52:RCIN2<br/>53:RCIN3<br/>54:RCIN4<br/>55:RCIN5<br/>56:RCIN6<br/>57:RCIN7<br/>58:RCIN8<br/>59:RCIN9<br/>60:RCIN10<br/>61:RCIN11<br/>62:RCIN12<br/>63:RCIN13<br/>64:RCIN14<br/>65:RCIN15<br/>66:RCIN16<br/>67:Ignition<br/>68:Choke<br/>69:Starter<br/>70:Throttle<br/> |  |  | Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
RC11_MAX | 1900 | 1 | PWM | 800 - 2200 |  | RC max PWM | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC11_MIN | 1100 | 1 | PWM | 800 - 2200 |  | RC min PWM | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC11_REV | 1 | 1 | list | -1:Reversed<br/>1:Normal<br/> |  | RC reversed | Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
RC11_TRIM | 0 | 1 | PWM | 800 - 2200 |  |  | RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC2_DZ | 30 | 1 | PWM | 0 - 200 |  | RC dead-zone | dead zone around trim or bottom
RC2_MAX | 1983 | 1 | PWM | 800 - 2200 |  | RC max PWM | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC2_MIN | 996 | 1 | PWM | 800 - 2200 |  | RC min PWM | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC2_REV | 1 | 1 | list | -1:Reversed<br/>1:Normal<br/> |  | RC reversed | Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
RC2_TRIM | 1492 | 1 | PWM | 800 - 2200 |  |  | RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC3_DZ | 30 | 1 | PWM | 0 - 200 |  | RC dead-zone | dead zone around trim or bottom
RC3_MAX | 1982 | 1 | PWM | 800 - 2200 |  | RC max PWM | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC3_MIN | 996 | 1 | PWM | 800 - 2200 |  | RC min PWM | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC3_REV | 1 | 1 | list | -1:Reversed<br/>1:Normal<br/> |  | RC reversed | Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
RC3_TRIM | 1000 | 1 | PWM | 800 - 2200 |  |  | RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC4_DZ | 40 | 1 | PWM | 0 - 200 |  | RC dead-zone | dead zone around trim or bottom
RC4_MAX | 1981 | 1 | PWM | 800 - 2200 |  | RC max PWM | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC4_MIN | 992 | 1 | PWM | 800 - 2200 |  | RC min PWM | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC4_REV | 1 | 1 | list | -1:Reversed<br/>1:Normal<br/> |  | RC reversed | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC4_TRIM | 1486 | 1 | PWM | 800 - 2200 |  |  | RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC5_DZ | 0 | 1 | PWM | 0 - 200 |  |  | dead zone around trim or bottom
RC5_FUNCTION | 0 | 1 | list | 0:Disabled<br/>1:RCPassThru<br/>2:Flap<br/>3:Flap_auto<br/>4:Aileron<br/>6:mount_pan<br/>7:mount_tilt<br/>8:mount_roll<br/>9:mount_open<br/>10:camera_trigger<br/>11:release<br/>12:mount2_pan<br/>13:mount2_tilt<br/>14:mount2_roll<br/>15:mount2_open<br/>16:DifferentialSpoiler1<br/>17:DifferentialSpoiler2<br/>18:AileronWithInput<br/>19:Elevator<br/>20:ElevatorWithInput<br/>21:Rudder<br/>24:Flaperon1<br/>25:Flaperon2<br/>26:GroundSteering<br/>27:Parachute<br/>28:EPM<br/>29:LandingGear<br/>30:EngineRunEnable<br/>31:HeliRSC<br/>32:HeliTailRSC<br/>33:Motor1<br/>34:Motor2<br/>35:Motor3<br/>36:Motor4<br/>37:Motor5<br/>38:Motor6<br/>39:Motor7<br/>40:Motor8<br/>51:RCIN1<br/>52:RCIN2<br/>53:RCIN3<br/>54:RCIN4<br/>55:RCIN5<br/>56:RCIN6<br/>57:RCIN7<br/>58:RCIN8<br/>59:RCIN9<br/>60:RCIN10<br/>61:RCIN11<br/>62:RCIN12<br/>63:RCIN13<br/>64:RCIN14<br/>65:RCIN15<br/>66:RCIN16<br/>67:Ignition<br/>68:Choke<br/>69:Starter<br/>70:Throttle<br/> |  |  | Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
RC5_MAX | 1982 | 1 | PWM | 800 - 2200 |  | RC max PWM | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC5_MIN | 992 | 1 | PWM | 800 - 2200 |  | RC min PWM | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC5_REV | 1 | 1 | list | -1:Reversed<br/>1:Normal<br/> |  | RC reversed | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC5_TRIM | 993 | 1 | PWM | 800 - 2200 |  |  | RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC6_DZ | 0 | 1 | PWM | 0 - 200 |  | RC dead-zone | dead zone around trim or bottom
RC6_FUNCTION | 0 | 1 | list | 0:Disabled<br/>1:RCPassThru<br/>2:Flap<br/>3:Flap_auto<br/>4:Aileron<br/>6:mount_pan<br/>7:mount_tilt<br/>8:mount_roll<br/>9:mount_open<br/>10:camera_trigger<br/>11:release<br/>12:mount2_pan<br/>13:mount2_tilt<br/>14:mount2_roll<br/>15:mount2_open<br/>16:DifferentialSpoiler1<br/>17:DifferentialSpoiler2<br/>18:AileronWithInput<br/>19:Elevator<br/>20:ElevatorWithInput<br/>21:Rudder<br/>24:Flaperon1<br/>25:Flaperon2<br/>26:GroundSteering<br/>27:Parachute<br/>28:EPM<br/>29:LandingGear<br/>30:EngineRunEnable<br/>31:HeliRSC<br/>32:HeliTailRSC<br/>33:Motor1<br/>34:Motor2<br/>35:Motor3<br/>36:Motor4<br/>37:Motor5<br/>38:Motor6<br/>39:Motor7<br/>40:Motor8<br/>51:RCIN1<br/>52:RCIN2<br/>53:RCIN3<br/>54:RCIN4<br/>55:RCIN5<br/>56:RCIN6<br/>57:RCIN7<br/>58:RCIN8<br/>59:RCIN9<br/>60:RCIN10<br/>61:RCIN11<br/>62:RCIN12<br/>63:RCIN13<br/>64:RCIN14<br/>65:RCIN15<br/>66:RCIN16<br/>67:Ignition<br/>68:Choke<br/>69:Starter<br/>70:Throttle<br/> |  |  | Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
RC6_MAX | 1985 | 1 | PWM | 800 - 2200 |  | RC max PWM | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC6_MIN | 992 | 1 | PWM | 800 - 2200 |  | RC min PWM | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC6_REV | 1 | 1 | list | -1:Reversed<br/>1:Normal<br/> |  | RC reversed | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC6_TRIM | 992 | 1 | PWM | 800 - 2200 |  |  | RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC7_DZ | 0 | 1 | PWM | 0 - 200 |  | RC dead-zone | dead zone around trim or bottom
RC7_FUNCTION | 0 | 1 | list | 0:Disabled<br/>1:RCPassThru<br/>2:Flap<br/>3:Flap_auto<br/>4:Aileron<br/>6:mount_pan<br/>7:mount_tilt<br/>8:mount_roll<br/>9:mount_open<br/>10:camera_trigger<br/>11:release<br/>12:mount2_pan<br/>13:mount2_tilt<br/>14:mount2_roll<br/>15:mount2_open<br/>16:DifferentialSpoiler1<br/>17:DifferentialSpoiler2<br/>18:AileronWithInput<br/>19:Elevator<br/>20:ElevatorWithInput<br/>21:Rudder<br/>24:Flaperon1<br/>25:Flaperon2<br/>26:GroundSteering<br/>27:Parachute<br/>28:EPM<br/>29:LandingGear<br/>30:EngineRunEnable<br/>31:HeliRSC<br/>32:HeliTailRSC<br/>33:Motor1<br/>34:Motor2<br/>35:Motor3<br/>36:Motor4<br/>37:Motor5<br/>38:Motor6<br/>39:Motor7<br/>40:Motor8<br/>51:RCIN1<br/>52:RCIN2<br/>53:RCIN3<br/>54:RCIN4<br/>55:RCIN5<br/>56:RCIN6<br/>57:RCIN7<br/>58:RCIN8<br/>59:RCIN9<br/>60:RCIN10<br/>61:RCIN11<br/>62:RCIN12<br/>63:RCIN13<br/>64:RCIN14<br/>65:RCIN15<br/>66:RCIN16<br/>67:Ignition<br/>68:Choke<br/>69:Starter<br/>70:Throttle<br/> |  |  | Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
RC7_MAX | 1900 | 1 | PWM | 800 - 2200 |  | RC max PWM | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC7_MIN | 1100 | 1 | PWM | 800 - 2200 |  | RC min PWM | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC7_REV | 1 | 1 | list | -1:Reversed<br/>1:Normal<br/> |  | RC reversed | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC7_TRIM | 1498 | 1 | PWM | 800 - 2200 |  |  | RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC8_DZ | 0 | 1 | PWM | 0 - 200 |  | RC dead-zone | dead zone around trim or bottom
RC8_FUNCTION | 0 | 1 | list | 0:Disabled<br/>1:RCPassThru<br/>2:Flap<br/>3:Flap_auto<br/>4:Aileron<br/>6:mount_pan<br/>7:mount_tilt<br/>8:mount_roll<br/>9:mount_open<br/>10:camera_trigger<br/>11:release<br/>12:mount2_pan<br/>13:mount2_tilt<br/>14:mount2_roll<br/>15:mount2_open<br/>16:DifferentialSpoiler1<br/>17:DifferentialSpoiler2<br/>18:AileronWithInput<br/>19:Elevator<br/>20:ElevatorWithInput<br/>21:Rudder<br/>24:Flaperon1<br/>25:Flaperon2<br/>26:GroundSteering<br/>27:Parachute<br/>28:EPM<br/>29:LandingGear<br/>30:EngineRunEnable<br/>31:HeliRSC<br/>32:HeliTailRSC<br/>33:Motor1<br/>34:Motor2<br/>35:Motor3<br/>36:Motor4<br/>37:Motor5<br/>38:Motor6<br/>39:Motor7<br/>40:Motor8<br/>51:RCIN1<br/>52:RCIN2<br/>53:RCIN3<br/>54:RCIN4<br/>55:RCIN5<br/>56:RCIN6<br/>57:RCIN7<br/>58:RCIN8<br/>59:RCIN9<br/>60:RCIN10<br/>61:RCIN11<br/>62:RCIN12<br/>63:RCIN13<br/>64:RCIN14<br/>65:RCIN15<br/>66:RCIN16<br/>67:Ignition<br/>68:Choke<br/>69:Starter<br/>70:Throttle<br/> |  |  | Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
RC8_MAX | 1900 | 1 | PWM | 800 - 2200 |  | RC max PWM | RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC8_MIN | 1100 | 1 | PWM | 800 - 2200 |  | RC min PWM | RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RC8_REV | 1 | 1 | list | -1:Reversed<br/>1:Normal<br/> |  | RC reversed | Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel.
RC8_TRIM | 1498 | 1 | PWM | 800 - 2200 |  |  | RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
RCMAP_PITCH | 2 | 1 |  | |  |  | Pitch channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Pitch is normally on channel 2, but you can move it to any channel with this parameter. Reboot is required for changes to take effect.
RCMAP_ROLL | 1 | 1 |  | |  |  | Roll channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Roll is normally on channel 1, but you can move it to any channel with this parameter. Reboot is required for changes to take effect.
RCMAP_THROTTLE | 3 | 1 |  | |  |  | Throttle channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Throttle is normally on channel 3, but you can move it to any channel with this parameter. Warning APM 2.X: Changing the throttle channel could produce unexpected fail-safe results if connection between receiver and on-board PPM Encoder is lost. Disabling on-board PPM Encoder is recommended. Reboot is required for changes to take effect.
RCMAP_YAW | 4 | 1 |  | |  |  | Yaw channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Yaw (also known as rudder) is normally on channel 4, but you can move it to any channel with this parameter. Reboot is required for changes to take effect.
RELAY_PIN | 13 | 1 | list | -1:Disabled<br/>13:APM2 A9 pin<br/>47:APM1 relay<br/>111:PX4 FMU Relay1<br/>112:PX4 FMU Relay2<br/>113:PX4IO Relay1<br/>50:Pixhawk AUXOUT1<br/>114:PX4IO Relay2<br/>51:Pixhawk AUXOUT2<br/>115:PX4IO ACC1<br/>52:Pixhawk AUXOUT3<br/>116:PX4IO ACC2<br/>53:Pixhawk AUXOUT4<br/>54:Pixhawk AUXOUT5<br/>55:Pixhawk AUXOUT6<br/> |  |  | Digital pin number for first relay control. This is the pin used for camera control.
RELAY_PIN2 | -1 | 1 | list | -1:Disabled<br/>13:APM2 A9 pin<br/>47:APM1 relay<br/>111:PX4 FMU Relay1<br/>112:PX4 FMU Relay2<br/>113:PX4IO Relay1<br/>50:Pixhawk AUXOUT1<br/>114:PX4IO Relay2<br/>51:Pixhawk AUXOUT2<br/>115:PX4IO ACC1<br/>52:Pixhawk AUXOUT3<br/>116:PX4IO ACC2<br/>53:Pixhawk AUXOUT4<br/>54:Pixhawk AUXOUT5<br/>55:Pixhawk AUXOUT6<br/> |  |  | Digital pin number for 2nd relay control.
RNGFND_FUNCTION | 0 | 1 | list | 0:Linear<br/>1:Inverted<br/>2:Hyperbolic<br/> |  |  | Control over what function is used to calculate distance. For a linear function, the distance is (MAV_PARAM_GROUP.ARDUCOPTER,voltage-offset)*scaling. For a inverted function the distance is (MAV_PARAM_GROUP.ARDUCOPTER,offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(MAV_PARAM_GROUP.ARDUCOPTER,voltage-offset). The functions return the distance in meters.
RNGFND_GAIN | 0.8 | 1 | unknown | 0.01 - 2.0 |  |  | Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter
RNGFND_MAX_CM | 700 | 1 | cm | |  |  | Maximum distance in centimeters that rangefinder can reliably read
RNGFND_MIN_CM | 20 | 1 | cm | |  |  | Minimum distance in centimeters that rangefinder can reliably read
RNGFND_OFFSET | 0 | 1 | volt | |  |  | Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM and I2C Lidars
RNGFND_PIN | -1 | 1 | list | -1:NotUsed<br/>0:APM2-A0<br/>64:APM1-airspeed port<br/>1:APM2-A1<br/>2:APM2-A2<br/>3:APM2-A3<br/>4:APM2-A4<br/>5:APM2-A5<br/>6:APM2-A6<br/>7:APM2-A7<br/>8:APM2-A8<br/>9:APM2-A9<br/>11:PX4-airspeed port<br/>15:Pixhawk-airspeed port<br/> |  |  | Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
RNGFND_RMETRIC | 1 | 1 | list | 0:No<br/>1:Yes<br/> |  |  | This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (MAV_PARAM_GROUP.ARDUCOPTER,such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
RNGFND_SCALING | 3 | 0.001 | m/volt | |  | Rangefinder scaling | Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
RNGFND_SETTLE_MS | 0 | 1 | unknown | |  |  | Coming soon
RNGFND_STOP_PIN | -1 | 1 | list | -1:NotUsed<br/>112:PX4 FMU Relay2<br/>113:PX4IO,Relay1<br/>50:Pixhawk AUXOUT1<br/>114:PX4IO Relay2<br/>51:Pixhawk AUXOUT2<br/>115:PX4IO ACC1<br/>52:Pixhawk AUXOUT3,53:Pixhawk,AUXOUT4,54:Pixhawk AUXOUT5<br/>116:PX4IO ACC2<br/>55:Pixhawk AUXOUT6<br/>111:PX4 FMU Relay1<br/> |  |  | Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
RNGFND_TYPE | 0 | 1 | list | 0:None<br/>1:Analog<br/>2:APM2-MaxbotixI2C<br/>3:APM2-PulsedLightI2C<br/>4:PX4-I2C<br/>5:PX4-PWM<br/>6:BBB-PRU<br/>7:LightWareI2C<br/>8:LightWareSerial<br/>9:Bebop<br/>10:MAVLink<br/>12:LeddarOne<br/> |  |  | What type of rangefinder device that is connected
RSSI_PIN | -1 | 1 | unknown | |  |  | Coming soon
RSSI_RANGE | 5 | 1 | unknown | |  |  | Coming soon
RTL_ALT | 1500 | 1 | cm | 0 - 8000 |  |  | The minimum relative altitude the model will move to before Returning to Launch. Set to zero to return at current altitude.
RTL_ALT_FINAL | 0 | 1 | cm | -1 - 1000 |  |  | This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission. Set to zero to land.
RTL_LOIT_TIME | 5000 | 1 | m/s | 0 - 60000 |  |  | Time (in milliseconds) to loiter above home before beginning final descent
SCHED_DEBUG | 0 | 1 | list | 0:Disabled<br/>2:ShowSlips<br/>3:ShowOverruns<br/> |  | Scheduler debug level |" Set to non-zero to enable scheduler debug messages. When set to show ""Slips"" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table."
SERIAL0_BAUD | 115 | 1 | list | 1:1200<br/>2:2400<br/>19:19200<br/>115:115200<br/>4:4800<br/>500:500000<br/>38:38400<br/>9:9600<br/>57:57600<br/>921:921600<br/>1500:1500000<br/>111:111100<br/> |  |  | The baud rate used on the USB console. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
SERIAL1_BAUD | 57 | 1 | list | 1:1200<br/>2:2400<br/>19:19200<br/>115:115200<br/>4:4800<br/>500:500000<br/>38:38400<br/>9:9600<br/>57:57600<br/>921:921600<br/>1500:1500000<br/>111:111100<br/> |  |  | The baud rate used on the Telem1 port. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
SIMPLE | 18 | 1 | unknown | |  |  | Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode)
SR0_EXT_STAT | 2 | 1 | Hz | 0 - 10 |  |  | Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
SR0_EXTRA1 | 4 | 1 | Hz | 0 - 10 |  |  | Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
SR0_EXTRA2 | 4 | 1 | Hz | 0 - 10 |  |  | Stream rate of VFR_HUD to ground station
SR0_EXTRA3 | 2 | 1 | Hz | 0 - 10 |  |  | Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
SR0_PARAMS | 10 | 1 | Hz | 0 - 10 |  |  | Stream rate of PARAM_VALUE to ground station
SR0_POSITION | 2 | 1 | Hz | 0 - 10 |  |  | Stream rate of GLOBAL_POSITION_INT to ground station
SR0_RAW_CTRL | 1 | 1 | Hz | 0 - 10 |  |  | Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
SR0_RAW_SENS | 2 | 1 | Hz | 0 - 10 |  |  | Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
SR0_RC_CHAN | 2 | 1 | Hz | 0 - 10 |  |  | Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station
SR1_EXT_STAT | 2 | 1 | Hz | 0 - 10 |  |  | Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
SR1_EXTRA1 | 2 | 1 | Hz | 0 - 10 |  |  | Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
SR1_EXTRA2 | 2 | 1 | Hz | 0 - 10 |  |  | Stream rate of VFR_HUD to ground station
SR1_EXTRA3 | 2 | 1 | Hz | 0 - 10 |  |  | Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
SR1_PARAMS | 0 | 1 | Hz | 0 - 10 |  |  | Stream rate of PARAM_VALUE to ground station
SR1_POSITION | 2 | 1 | Hz | 0 - 10 |  |  | Stream rate of GLOBAL_POSITION_INT to ground station
SR1_RAW_CTRL | 2 | 1 | Hz | 0 - 10 |  |  | Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
SR1_RAW_SENS | 2 | 1 | Hz | 0 - 10 |  |  | Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
SR1_RC_CHAN | 2 | 1 | Hz | 0 - 10 |  |  | Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station
STB_PIT_P | 0.016 | 1 | unknown | |  |  | Coming soon
STB_RLL_P | 0.016 | 1 | unknown | |  |  | Coming soon
STB_YAW_P | 4 | 1 | unknown | |  |  | Coming soon
SUPER_SIMPLE | 5 | 1 | list | 0:Disabled<br/>1:Mode1<br/>2:Mode2<br/>3:Mode1+2<br/>4:Mode3<br/>5:Mode1+3<br/>6:Mode2+3<br/>7:Mode1+2+3<br/>8:Mode4<br/>9:Mode1+4<br/>10:Mode2+4<br/>11:Mode1+2+4<br/>12:Mode3+4<br/>13:Mode1+3+4<br/>14:Mode2+3+4<br/>15:Mode1+2+3+4<br/>16:Mode5<br/>17:Mode1+5<br/>18:Mode2+5<br/>19:Mode1+2+5<br/>20:Mode3+5<br/>21:Mode1+3+5<br/>22:Mode2+3+5<br/>23:Mode1+2+3+5<br/>24:Mode4+5<br/>25:Mode1+4+5<br/>26:Mode2+4+5<br/>27:Mode1+2+4+5<br/>28:Mode3+4+5<br/>29:Mode1+3+4+5<br/>30:Mode2+3+4+5<br/>31:Mode1+2+3+4+5<br/>32:Mode6<br/>33:Mode1+6<br/>34:Mode2+6<br/>35:Mode1+2+6<br/>36:Mode3+6<br/>37:Mode1+3+6<br/>38:Mode2+3+6<br/>39:Mode1+2+3+6<br/>40:Mode4+6<br/>41:Mode1+4+6<br/>42:Mode2+4+6<br/>43:Mode1+2+4+6<br/>44:Mode3+4+6<br/>45:Mode1+3+4+6<br/>46:Mode2+3+4+6<br/>47:Mode1+2+3+4+6<br/>48:Mode5+6<br/>49:Mode1+5+6<br/>50:Mode2+5+6<br/>51:Mode1+2+5+6<br/>52:Mode3+5+6<br/>53:Mode1+3+5+6<br/>54:Mode2+3+5+6<br/>55:Mode1+2+3+5+6<br/>56:Mode4+5+6<br/>57:Mode1+4+5+6<br/>58:Mode2+4+5+6<br/>59:Mode1+2+4+5+6<br/>60:Mode3+4+5+6<br/>61:Mode1+3+4+5+6<br/>62:Mode2+3+4+5+6<br/>63:Mode1+2+3+4+5+6<br/> |  |  | Bitmask to enable Super Simple mode for some flight modes. Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will disable Super Simple Mode
SYSID_MYGCS | 255 | 1 | list | 252:AP Planner 2<br/>255:Mission Planner and DroidPlanner<br/> |  | My ground station number | Allows restricting radio overrides to only come from my ground station
SYSID_SW_MREV | 120 | 1 |  | | V | Eeprom format version number | This value is incremented when changes are made to the eeprom format
SYSID_SW_TYPE | 10 | 1 | list | 0:ArduPlane<br/>4:AntennaTracker<br/>20:Rover<br/>10:Copter<br/> |  |  | This is used by the ground station to recognise the software type (MAV_PARAM_GROUP.ARDUCOPTER,eg ArduPlane vs ArduCopter)
SYSID_THISMAV | 1 | 1 | unknown | 1 - 255 |  |  | Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
TELEM_DELAY | 0 | 1 | second | 0 - 30 |  |  | The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
THR_ACCEL_D | 0 | 1 | unknown | |  |  | Coming soon
THR_ACCEL_I | 1 | 1 | unknown | |  |  | Coming soon
THR_ACCEL_IMAX | 800 | 1 | unknown | |  |  | Coming soon
THR_ACCEL_P | 0.5 | 1 | unknown | |  |  | Coming soon
THR_ALT_P | 1 | 1 | unknown | |  |  | Coming soon
THR_DZ | 100 | 1 | PWM | 0 - 300 |  |  | The deadzone above and below mid throttle. Used in AltHold, Loiter, PosHold flight modes
THR_MAX | 1000 | 1 | unknown | |  |  | Coming soon
THR_MID | 510 | 1 | unknown | |  |  | Coming soon
THR_MIN | 130 | 1 | unknown | |  |  | Coming soon
THR_RATE_P | 6 | 1 | unknown | |  |  | Coming soon
TRIM_THROTTLE | 724 | 1 | unknown | |  |  | Coming soon
TUNE | 1 | 1 | list | 0:None<br/>1:Stab_Roll/Pitch_kP<br/>3:Stab_Yaw_kP<br/>4:Rate_Roll/Pitch_kP<br/>5:Rate_Roll/Pitch_kI<br/>6:Rate_Yaw_kP<br/>7:Throttle_Rate_kP<br/>10:WP_Speed<br/>12:Loiter_Pos_kP<br/>13:Heli_Ext_Gyro<br/>14:Altitude_Hold_kP<br/>17:OF_Loiter_kP<br/>18:OF_Loiter_kI<br/>19:OF_Loiter_kD<br/>21:Rate_Roll/Pitch_kD<br/>22:Velocity_XY_kP<br/>25:Acro_RollPitch_kP<br/>26:Rate_Yaw_kD<br/>28:Velocity_XY_kI<br/>34:Throttle_Accel_kP<br/>35:Throttle_Accel_kI<br/>36:Throttle_Accel_kD<br/>38:Declination<br/>39:Circle_Rate<br/>40:Acro_Yaw_kP<br/>41:RangeFinder_Gain<br/>42:Loiter_Speed<br/>46:Rate_Pitch_kP<br/>47:Rate_Pitch_kI<br/>48:Rate_Pitch_kD<br/>49:Rate_Roll_kP<br/>50:Rate_Roll_kI<br/>51:Rate_Roll_kD<br/>52:Rate_Pitch_FF<br/>53:Rate_Roll_FF<br/>54:Rate_Yaw_FF<br/> |  |  | Controls which parameters (MAV_PARAM_GROUP.ARDUCOPTER,normally PID gains) are being tuned with transmitter's channel 6 knob
TUNE_HIGH | 32 | 1 | unknown | 0 - 32767 |  |  | The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
TUNE_LOW | 0 | 1 | unknown | 0 - 32767 |  |  | The minimum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
WP_YAW_BEHAVIOR | 2 | 1 | list | 0:NeverChangeYaw<br/>1:FaceNextWaypoint<br/>2:FaceNextWaypointExceptRTL<br/>3:FaceAlongGPScourse<br/> |  |  | Determines how the autopilot controls the yaw during missions and RTL
WPNAV_ACCEL | 100 | 1 | cm/s/s | 50 - 500 |  |  | Defines the horizontal acceleration in CENTIMETER/s/s used during missions
WPNAV_ACCEL_Z | 100 | 1 | cm/s/s | 50 - 500 |  |  | Defines the vertical acceleration in CENTIMETER/s/s used during missions
WPNAV_LOIT_JERK | 1000 | 1 | cm/s/s/s | 500 - 5000 |  |  | Loiter maximum jerk in CENTIMETER/s/s/s
WPNAV_LOIT_SPEED | 1000 | 1 | cm/s | 20 - 2000 |  |  | Defines the maximum speed in CENTIMETER/s which the aircraft will travel horizontally while in loiter mode
WPNAV_RADIUS | 200 | 1 | cm | 100 - 1000 |  |  | Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
WPNAV_SPEED | 500 | 1 | cm/s | 0 - 2000 |  |  | Defines the speed in CENTIMETER/s which the aircraft will attempt to maintain horizontally during a WP mission
WPNAV_SPEED_DN | 150 | 1 | cm/s | 10 - 500 |  |  | Defines the speed in CENTIMETER/s which the aircraft will attempt to maintain while descending during a WP mission
WPNAV_SPEED_UP | 250 | 1 | cm/s | 10 - 1000 |  | Waypoint Climb Speed Target | Defines the speed in CENTIMETER/s which the aircraft will attempt to maintain while climbing during a WP mission