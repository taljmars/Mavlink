package com.dronegcs.mavlink.is.protocol.msg_metadata.enums;

import com.dronegcs.mavlink.is.units.Range;
import com.opencsv.CSVWriter;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public enum MAV_PARAM_COPTER implements MAV_PARAM_I {

    ACRO_BAL_PITCH(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,0.1,MAV_PARAM_UNIT.UNKNOWN,new Range(0,3),false,"Acro Balance Pitch","rate at which pitch angle returns to level in acro mode. A higher value causes the vehicle to return to level faster."),
    ACRO_BAL_ROLL(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,0.1,MAV_PARAM_UNIT.UNKNOWN,new Range(0,3),false,"Acro Balance Roll","rate at which roll angle returns to level in acro mode. A higher value causes the vehicle to return to level faster."),
    ACRO_EXPO(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.3,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    ACRO_RP_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,4.5,1,MAV_PARAM_UNIT.NONE,false,"","Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode. Higher values mean faster rate of rotation."),
    ACRO_TRAINER(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,2,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Leveling");put(2,"LevelingAndLimited");}},false,"Acro Trainer",""),
    ACRO_YAW_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,4.5,1,MAV_PARAM_UNIT.NONE,false,"","Converts pilot yaw input into a desired rate of rotation in ACRO, Stabilize and SPORT modes. Higher values mean faster rate of rotation."),
    AHRS_COMP_BETA(MAV_PARAM_GROUP_COPTER.AHRS,0.1,0.01,MAV_PARAM_UNIT.NONE,new Range(0.001,0.5),false,"AHRS Velocity Complementary Filter Beta Coefficient","This controls the time constant for the cross-over frequency used to fuse AHRS (MAV_PARAM_GROUP.ARDUCOPTER,airspeed and heading) and GPS data to estimate ground velocity. Time constant is 0.1/beta. A larger time constant will use GPS data less and a small time constant will use air data less."),
    AHRS_GPS_GAIN(MAV_PARAM_GROUP_COPTER.AHRS,1,0.01,MAV_PARAM_UNIT.NONE,new Range(0.0,1.0),false,"AHRS GPS gain","This controls how how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0."),
    AHRS_GPS_MINSATS(MAV_PARAM_GROUP_COPTER.AHRS,6,1,MAV_PARAM_UNIT.NONE,new Range(0,10),false,"AHRS GPS Minimum satellites","Minimum number of satellites visible to use GPS for velocity based corrections attitude correction. This defaults to 6, which is about the point at which the velocity numbers from a GPS become too unreliable for accurate correction of the accelerometers."),
    AHRS_GPS_USE(MAV_PARAM_GROUP_COPTER.AHRS,1,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Enabled");}},false,"AHRS use GPS for navigation","This controls whether to use dead-reckoning or GPS based navigation. If set to 0 then the GPS won’t be used for navigation, and only dead reckoning will be used. A value of zero should never be used for normal flight. Currently this affects only the DCM-based AHRS: the EKF uses GPS whenever it is available."),
    AHRS_ORIENTATION(MAV_PARAM_GROUP_COPTER.AHRS,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"None");put(1,"Yaw45");put(2,"Yaw90");put(3,"Yaw135");put(4,"Yaw180");put(5,"Yaw225");put(6,"Yaw270");put(7,"Yaw315");put(8,"Roll180");put(9,"Roll180Yaw45");put(10,"Roll180Yaw90");put(11,"Roll180Yaw135");put(12,"Pitch180");put(13,"Roll180Yaw225");put(14,"Roll180Yaw270");put(15,"Roll180Yaw315");put(16,"Roll90");put(17,"Roll90Yaw45");put(18,"Roll90Yaw90");put(19,"Roll90Yaw135");put(20,"Roll270");put(21,"Roll270Yaw45");put(22,"Roll270Yaw90");put(23,"Roll270Yaw136");put(24,"Pitch90");put(25,"Pitch270");put(26,"Pitch180Yaw90");put(27,"Pitch180Yaw270");put(28,"Roll90Pitch90");put(29,"Roll180Pitch90");put(30,"Roll270Pitch90");put(31,"Roll90Pitch180");put(32,"Roll270Pitch180");put(33,"Roll90Pitch270");put(34,"Roll180Pitch270");put(35,"Roll270Pitch270");put(36,"Roll90Pitch180Yaw90");put(37,"Roll90Yaw270");}},false,"Board Orientation","Overall board orientation relative to the standard orientation for the board type. This rotates the IMU and compass readings to allow the board to be oriented in your vehicle at any 90 or 45 degree angle. This option takes affect on next boot. After changing you will need to re-level your vehicle."),
    AHRS_RP_P(MAV_PARAM_GROUP_COPTER.AHRS,0.1,0.01,MAV_PARAM_UNIT.UNKNOWN,new Range(0.1,0.4),false,"AHRS RP_P","This controls how fast the accelerometers correct the attitude"),
    AHRS_TRIM_X(MAV_PARAM_GROUP_COPTER.AHRS,0.0004882812,0.01,MAV_PARAM_UNIT.RADIANS,new Range(-0.1745,0.1745),false,"AHRS Trim Roll","Compensates for the roll angle difference between the control board and the frame. Positive values make the vehicle roll right."),
    AHRS_TRIM_Y(MAV_PARAM_GROUP_COPTER.AHRS,0.02518386,0.01,MAV_PARAM_UNIT.RADIANS,new Range(-0.1745,0.1745),false,"AHRS Trim Pitch","Compensates for the pitch angle difference between the control board and the frame. Positive values make the vehicle pitch up/back."),
    AHRS_TRIM_Z(MAV_PARAM_GROUP_COPTER.AHRS,0,0.01,MAV_PARAM_UNIT.RADIANS,new Range(-0.1745,0.1745),false,"AHRS Trim Yaw","Not Used"),
    AHRS_WIND_MAX(MAV_PARAM_GROUP_COPTER.AHRS,0,1,MAV_PARAM_UNIT.METER_PER_SECOND,new Range(0,127),false,"Maximum wind","This sets the maximum allowable difference between ground speed and airspeed. This allows the plane to cope with a failing airspeed sensor. A value of zero means to use the airspeed as is."),
    AHRS_YAW_P(MAV_PARAM_GROUP_COPTER.AHRS,0.1,1,MAV_PARAM_UNIT.UNKNOWN,new Range(0.1,0.4),false,"","This controls the weight the compass or GPS has on the heading. A higher value means the heading will track the yaw source (MAV_PARAM_GROUP.ARDUCOPTER,GPS or compass) more rapidly."),
    ANGLE_MAX(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,4500,1,MAV_PARAM_UNIT.CENTIDEGREE,new Range(1000,8000),false,"Angle Max","Maximum lean angle in all flight modes"),
    ARMING_CHECK(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,94,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Enabled");put(-3,"SkipBaro");put(-5,"SkipCompass");put(-9,"SkipGPS");put(-17,"SkipINS");put(-33,"SkipParams/Rangefinder");put(-65,"SkipRC");put(127,"SkipVoltage");}},false,"","Allows enabling or disabling of pre-arming checks of receiver, accelerometer, barometer, compass and GPS"),
    ATC_ACCEL_RP_MAX(MAV_PARAM_GROUP_COPTER.ATC,0,1000,MAV_PARAM_UNIT.CENTIDEGREE_PER_SECOND_PER_SECOND,new HashMap<Integer, String>(){{put(0,"Disabled");put(9000,"VerySlow");put(18000,"Slow");put(36000,"Medium");put(54000,"Fast");}},false,"Acceleration Max for Roll/Pitch","Maximum acceleration in roll/pitch axis"),
    ATC_ACCEL_Y_MAX(MAV_PARAM_GROUP_COPTER.ATC,0,1000,MAV_PARAM_UNIT.CENTIDEGREE_PER_SECOND_PER_SECOND,new HashMap<Integer, String>(){{put(0,"Disabled");put(9000,"VerySlow");put(18000,"Slow");put(36000,"Medium");put(54000,"Fast");}},false,"Acceleration Max for Yaw","Maximum acceleration in yaw axis"),
    ATC_RATE_FF_ENAB(MAV_PARAM_GROUP_COPTER.ATC,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Enabled");}},false,"","Controls whether body-frame rate feedforward is enabled or disabled"),
    ATC_RATE_RP_MAX(MAV_PARAM_GROUP_COPTER.ATC,18000,1,MAV_PARAM_UNIT.DEGREE_PER_SECOND,new HashMap<Integer, String>(){{put(0,"Disabled");put(360,"Slow");put(720,"Medium");put(1080,"Fast");}},false,"Angular Velocity Max for Pitch/Roll","Maximum angular velocity in pitch/roll axis"),
    ATC_RATE_Y_MAX(MAV_PARAM_GROUP_COPTER.ATC,9000,1,MAV_PARAM_UNIT.DEGREE_PER_SECOND,new HashMap<Integer, String>(){{put(0,"Disabled");put(360,"Slow");put(720,"Medium");put(1080,"Fast");}},false,"Angular Velocity Max for Yaw","Maximum angular velocity in yaw axis"),
    ATC_SLEW_YAW(MAV_PARAM_GROUP_COPTER.ATC,1000,1,MAV_PARAM_UNIT.CENTIDEGREE_PER_SECOND,new Range(500,18000),false,"Yaw target slew rate","Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes"),
    BAROGLTCH_ACCEL(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1500,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    BAROGLTCH_DIST(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,500,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    BAROGLTCH_ENABLE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    BATT_AMP_OFFSET(MAV_PARAM_GROUP_COPTER.BATT,0,1,MAV_PARAM_UNIT.VOLT,false,"AMP offset","Voltage offset at zero current on current sensor"),
    BATT_AMP_PERVOLT(MAV_PARAM_GROUP_COPTER.BATT,17,1,MAV_PARAM_UNIT.AMPER_PER_VOLT,false,"Amps per volt","Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17."),
    BATT_AMP_PERVLT(MAV_PARAM_GROUP_COPTER.BATT,17,1,MAV_PARAM_UNIT.AMPER_PER_VOLT,false,"Amps per volt","Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17."),
    BATT_CAPACITY(MAV_PARAM_GROUP_COPTER.BATT,4000,1,MAV_PARAM_UNIT.MILLIAMPER_PER_HOUR,false,"Battery capacity","Capacity of the battery in mAh when full"),

    //Need update ->
    BATT_CURR_PIN(MAV_PARAM_GROUP_COPTER.BATT,3,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(-1,"Disabled");put(1,"A1");put(2,"A2");put(3,"Pixhawk");put(12,"A12");put(101,"PX4");}},false,"","Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk powered from the PM connector it should be set to 3."),
    BATT_MONITOR(MAV_PARAM_GROUP_COPTER.BATT,4,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(3,"Analog_Voltage_Only");put(4,"Analog_Voltage_and_Current");put(5,"SMBus");put(6,"Bebop");}},false,"Battery monitoring","Controls enabling monitoring of the battery's voltage and current"),
    BATT_VOLT_MULT(MAV_PARAM_GROUP_COPTER.BATT,10.1,1,MAV_PARAM_UNIT.UNKNOWN,false,"Voltage Multiplier","Used to convert the voltage of the voltage sensing pin (MAV_PARAM_GROUP.ARDUCOPTER,BATT_VOLT_PIN) to the actual battery's voltage (MAV_PARAM_GROUP.ARDUCOPTER,pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX4 using the PX4IO power supply this should be set to 1."),
    BATT_VOLT_PIN(MAV_PARAM_GROUP_COPTER.BATT,2,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(-1,"Disabled");put(0,"A0");put(1,"A1");put(2,"Pixhawk");put(13,"A13");put(100,"PX4");}},false,"Battery Voltage sensing pin","Setting this to 0 ~ 13 will enable battery voltage sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2."),
    BATT_VOLT2_MULT(MAV_PARAM_GROUP_COPTER.BATT,1,1,MAV_PARAM_UNIT.UNKNOWN,false,"Voltage Multiplier","Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery’s voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1"),
    BATT_VOLT2_PIN(MAV_PARAM_GROUP_COPTER.BATT,-1,1,MAV_PARAM_UNIT.UNKNOWN,false,"Battery Voltage sensing pin","Sets the analog input pin that should be used for voltage monitoring."),
    CAM_DURATION(MAV_PARAM_GROUP_COPTER.CAM,10,1,MAV_PARAM_UNIT.DECISECOND,new Range(0,50),false,"Duration that shutter is held open","How long the shutter will be held open in 10ths of a second (i.e. enter 10 for 1second, 50 for 5seconds)"),
    CAM_SERVO_OFF(MAV_PARAM_GROUP_COPTER.CAM,1100,1,MAV_PARAM_UNIT.PWM_PER_MICROSECOND,new Range(1000,2000),false,"Servo OFF PWM value","PWM value to move servo to when shutter is deactivated"),
    CAM_SERVO_ON(MAV_PARAM_GROUP_COPTER.CAM,1300,1,MAV_PARAM_UNIT.PWM_PER_MICROSECOND,new Range(1000,2000),false,"Servo ON PWM value","PWM value to move servo to when shutter is activated"),
    CAM_TRIGG_DIST(MAV_PARAM_GROUP_COPTER.CAM,0,1,MAV_PARAM_UNIT.METER,new Range(0,1000),false,"Camera trigger distance","Distance in meters between camera triggers. If this value is non-zero then the camera will trigger whenever the GPS position changes by this number of meters regardless of what mode the APM is in. Note that this parameter can also be set in an auto mission using the DO_SET_CAM_TRIGG_DIST command, allowing you to enable/disable the triggering of the camera during the flight."),
    CAM_TRIGG_TYPE(MAV_PARAM_GROUP_COPTER.CAM,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Servo");put(1,"Relay");}},false,"Camera shutter (trigger) type","how to trigger the camera to take a picture"),
    CH7_OPT(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,17,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"DoNothing");put(2,"Flip");put(3,"SimpleMode");put(4,"RTL");put(5,"SaveTrim");put(7,"SaveWP");put(9,"CameraTrigger");put(10,"RangeFinder");put(11,"Fence");put(13,"SuperSimpleMode");put(14,"AcroTrainer");put(15,"Sprayer");put(16,"Auto");put(17,"AutoTune");put(18,"Land");put(19,"EPM");put(21,"ParachuteEnable");put(22,"ParachuteRelease");put(23,"Parachute3pos");put(24,"AutoMissionReset");put(25,"AttConFeedForward");put(26,"AttConAccelLimits");put(27,"RetractMount");put(28,"Relay-On/Off");put(34,"Relay2-On/Off");put(35,"Relay3-On/Off");put(36,"Relay4-On/Off");put(29,"Landing-Gear");put(30,"Lost-Copter-Sound");put(31,"Motor-Emergency-Stop");put(32,"MotorInterlock");put(33,"Brake");put(37,"Throw");put(38,"Avoidance");}},false,"Channel 7 option","Select which function is performed when CH7 is above 1800 pwm"),
    CH8_OPT(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"DoNothing");put(2,"Flip");put(3,"SimpleMode");put(4,"RTL");put(5,"SaveTrim");put(7,"SaveWP");put(9,"CameraTrigger");put(10,"RangeFinder");put(11,"Fence");put(13,"SuperSimpleMode");put(14,"AcroTrainer");put(15,"Sprayer");put(16,"Auto");put(17,"AutoTune");put(18,"Land");put(19,"EPM");put(21,"ParachuteEnable");put(22,"ParachuteRelease");put(23,"Parachute3pos");put(24,"AutoMissionReset");put(25,"AttConFeedForward");put(26,"AttConAccelLimitsRetractMount");put(28,"Relay-On/Off");put(34,"Relay2-On/Off");put(35,"Relay3-On/Off");put(36,"Relay4-On/Off");put(29,"Landing-Gear");put(30,"Lost-Copter-Sound");put(31,"Motor-Emergency-Stop");put(32,"MotorInterlock");put(33,"Brake");put(37,"Throw");put(38,"Avoidance");}},false,"Channel 8 option","Select which function is performed when CH8 is above 1800 pwm"),
    CIRCLE_RADIUS(MAV_PARAM_GROUP_COPTER.CIRCLE,500,1,MAV_PARAM_UNIT.CENTIMETER,new Range(0,10000),false,"","Defines the radius of the circle the vehicle will fly when in Circle flight mode"),
    CIRCLE_RATE(MAV_PARAM_GROUP_COPTER.CIRCLE,20,1,MAV_PARAM_UNIT.DEGREE_PER_SECOND,new Range(-90,90),false,"","Circle mode's turn rate in deg/sec. Positive to turn clockwise, negative for counter clockwise"),
    COMPASS_AUTODEC(MAV_PARAM_GROUP_COPTER.COMPASS,1,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Enabled");}},false,"Auto Declination","Enable or disable the automatic calculation of the declination based on gps location"),
    COMPASS_DEC(MAV_PARAM_GROUP_COPTER.COMPASS,0.06941485,0.01,MAV_PARAM_UNIT.RADIANS,new Range(-3.142,3.142),false,"Compass declination","An angle to compensate between the true north and magnetic north"),
    COMPASS_EXTERNAL(MAV_PARAM_GROUP_COPTER.COMPASS,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Internal");put(1,"External");put(2,"ForcedExternal");}},false,"","Configure compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled."),
    COMPASS_LEARN(MAV_PARAM_GROUP_COPTER.COMPASS,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Internal-Learning");put(2,"EKF-Learning");}},false,"Learn compass offsets automatically","Enable or disable the automatic learning of compass offsets. You can enable learning either using a compass-only method that is suitable only for fixed wing aircraft or using the offsets learnt by the active EKF state estimator. If this option is enabled then the learnt offsets are saved when you disarm the vehicle."),
    COMPASS_MOT_X(MAV_PARAM_GROUP_COPTER.COMPASS,0,1,MAV_PARAM_UNIT.MILLIGAUSS_PER_AMPER,new Range(-1000,1000),false,"Motor interference compensation for body frame X axis","Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference"),
    COMPASS_MOT_Y(MAV_PARAM_GROUP_COPTER.COMPASS,0,1,MAV_PARAM_UNIT.MILLIGAUSS_PER_AMPER,new Range(-1000,1000),false,"Motor interference compensation for body frame Y axis","Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference"),
    COMPASS_MOT_Z(MAV_PARAM_GROUP_COPTER.COMPASS,0,1,MAV_PARAM_UNIT.MILLIGAUSS_PER_AMPER,new Range(-1000,1000),false,"Motor interference compensation for body frame Z axis","Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference"),
    COMPASS_MOTCT(MAV_PARAM_GROUP_COPTER.COMPASS,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"UseThrottle");put(2,"UseCurrent");}},false,"Motor interference compensation type","Set motor interference compensation type to disabled, throttle or current. Do not change manually."),
    COMPASS_OFS_X(MAV_PARAM_GROUP_COPTER.COMPASS,-98,1,MAV_PARAM_UNIT.MILLIGAUSS,new Range(-400,400),false,"Compass offsets in milligauss on the X axis","Offset to be added to the compass x-axis values to compensate for metal in the frame"),
    COMPASS_OFS_Y(MAV_PARAM_GROUP_COPTER.COMPASS,29,1,MAV_PARAM_UNIT.MILLIGAUSS,new Range(-400,400),false,"Compass offsets in milligauss on the Y axis","Offset to be added to the compass y-axis values to compensate for metal in the frame"),
    COMPASS_OFS_Z(MAV_PARAM_GROUP_COPTER.COMPASS,-18,1,MAV_PARAM_UNIT.MILLIGAUSS,new Range(-400,400),false,"Compass offsets in milligauss on the X axis","Offset to be added to the compass z-axis values to compensate for metal in the frame"),
    COMPASS_ORIENT(MAV_PARAM_GROUP_COPTER.COMPASS,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"None");put(1,"Yaw45");put(2,"Yaw90");put(3,"Yaw135");put(4,"Yaw180");put(5,"Yaw225");put(6,"Yaw270");put(7,"Yaw315");put(8,"Roll180");put(9,"Roll180Yaw45");put(10,"Roll180Yaw90");put(11,"Roll180Yaw135");put(12,"Pitch180");put(13,"Roll180Yaw225");put(14,"Roll180Yaw270");put(15,"Roll180Yaw315");put(16,"Roll90");put(17,"Roll90Yaw45");put(18,"Roll90Yaw90");put(19,"Roll90Yaw135");put(20,"Roll270");put(21,"Roll270Yaw45");put(22,"Roll270Yaw90");put(23,"Roll270Yaw136");put(24,"Pitch90");put(25,"Pitch270");put(26,"Pitch180Yaw90");put(27,"Pitch180Yaw270");put(28,"Roll90Pitch90");put(29,"Roll180Pitch90");put(30,"Roll270Pitch90");put(31,"Roll90Pitch180");put(32,"Roll270Pitch180");put(33,"Roll90Pitch270");put(34,"Roll180Pitch270");put(35,"Roll270Pitch270");put(36,"Roll90Pitch180Yaw90");put(37,"Roll90Yaw270");put(38,"Yaw293Pitch68Roll90");}},false,"","The orientation of the compass relative to the autopilot board. This will default to the right value for each board type, but can be changed if you have an external compass. See the documentation for your external compass for the right value. The correct orientation should give the X axis forward, the Y axis to the right and the Z axis down. So if your aircraft is pointing west it should show a positive value for the Y axis, and a value close to zero for the X axis. On a PX4 or Pixhawk with an external compass the correct value is zero if the compass is correctly oriented. NOTE: This orientation is combined with any AHRS_ORIENTATION setting."),
    COMPASS_USE(MAV_PARAM_GROUP_COPTER.COMPASS,1,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Enabled");}},false,"Use compass for yaw","Enable or disable the use of the compass (MAV_PARAM_GROUP.ARDUCOPTER,instead of the GPS) for determining heading"),
    DCM_CHECK_THRESH(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.8,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    EKF_CHECK_THRESH(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.8,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    ESC(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    FENCE_ACTION(MAV_PARAM_GROUP_COPTER.FENCE,1,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"ReportOnly");put(1,"RTL-or-Land");}},false,"","What action should be taken when fence is breached"),
    FENCE_ALT_MAX(MAV_PARAM_GROUP_COPTER.FENCE,100,1,MAV_PARAM_UNIT.METER,new Range(10,1000),false,"Fence Maximum Altitude","Maximum altitude allowed before geofence triggers"),
    FENCE_ENABLE(MAV_PARAM_GROUP_COPTER.FENCE,1,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Enabled");}},false,"","Allows you to enable (MAV_PARAM_GROUP.ARDUCOPTER,1) or disable (MAV_PARAM_GROUP.ARDUCOPTER,0) the fence functionality"),
    FENCE_MARGIN(MAV_PARAM_GROUP_COPTER.FENCE,2,1,MAV_PARAM_UNIT.METER,new Range(1,10),false,"Fence Margin","Distance that autopilot's should maintain from the fence to avoid a breach"),
    FENCE_RADIUS(MAV_PARAM_GROUP_COPTER.FENCE,300,1,MAV_PARAM_UNIT.METER,new Range(30,10000),false,"","Circle fence radius which when breached will cause an RTL"),
    FENCE_TYPE(MAV_PARAM_GROUP_COPTER.FENCE,1,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"None");put(1,"Altitude");put(2,"Circle");put(3,"AltitudeAndCircle");put(4,"Polygon");put(5,"AltitudeAndPolygon");put(6,"CircleAndPolygon");put(7,"All");}},false,"","Enabled fence types held as bitmask"),
    FLOW_ENABLE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Enabled");}},false,"","Setting this to Enabled(MAV_PARAM_GROUP.ARDUCOPTER,1) will enable optical flow. Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will disable optical flow"),
    FLTMODE1(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Stabilize");put(1,"Acro");put(2,"AltHold");put(3,"Auto");put(4,"Guided");put(5,"Loiter");put(6,"RTL");put(7,"Circle");put(9,"Land");put(11,"Drift");put(13,"Sport");put(14,"Flip");put(15,"AutoTune");put(16,"PosHold");put(17,"Brake");put(18,"Throw");put(19,"Avoid_ADSB");put(20,"Guided_NoGPS");}},false,"","Flight mode when Channel 5 pwm is <= 1230"),
    FLTMODE2(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Stabilize");put(1,"Acro");put(2,"AltHold");put(3,"Auto");put(4,"Guided");put(5,"Loiter");put(6,"RTL");put(7,"Circle");put(9,"Land");put(11,"Drift");put(13,"Sport");put(14,"Flip");put(15,"AutoTune");put(16,"PosHold");put(17,"Brake");put(18,"Throw");put(19,"Avoid_ADSB");put(20,"Guided_NoGPS");}},false,"","Flight mode when Channel 5 pwm is >1230, <= 1360"),
    FLTMODE3(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,16,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Stabilize");put(1,"Acro");put(2,"AltHold");put(3,"Auto");put(4,"Guided");put(5,"Loiter");put(6,"RTL");put(7,"Circle");put(9,"Land");put(11,"Drift");put(13,"Sport");put(14,"Flip");put(15,"AutoTune");put(16,"PosHold");put(17,"Brake");put(18,"Throw");put(19,"Avoid_ADSB");put(20,"Guided_NoGPS");}},false,"","Flight mode when Channel 5 pwm is >1360, <= 1490"),
    FLTMODE4(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,16,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Stabilize");put(1,"Acro");put(2,"AltHold");put(3,"Auto");put(4,"Guided");put(5,"Loiter");put(6,"RTL");put(7,"Circle");put(9,"Land");put(11,"Drift");put(13,"Sport");put(14,"Flip");put(15,"AutoTune");put(16,"PosHold");put(17,"Brake");put(18,"Throw");put(19,"Avoid_ADSB");put(20,"Guided_NoGPS");}},false,"","Flight mode when Channel 5 pwm is >1490, <= 1620"),
    FLTMODE5(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,6,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Stabilize");put(1,"Acro");put(2,"AltHold");put(3,"Auto");put(4,"Guided");put(5,"Loiter");put(6,"RTL");put(7,"Circle");put(9,"Land");put(11,"Drift");put(13,"Sport");put(14,"Flip");put(15,"AutoTune");put(16,"PosHold");put(17,"Brake");put(18,"Throw");put(19,"Avoid_ADSB");put(20,"Guided_NoGPS");}},false,"","Flight mode when Channel 5 pwm is >1620, <= 1749"),
    FLTMODE6(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,6,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Stabilize");put(1,"Acro");put(2,"AltHold");put(3,"Auto");put(4,"Guided");put(5,"Loiter");put(6,"RTL");put(7,"Circle");put(9,"Land");put(11,"Drift");put(13,"Sport");put(14,"Flip");put(15,"AutoTune");put(16,"PosHold");put(17,"Brake");put(18,"Throw");put(19,"Avoid_ADSB");put(20,"Guided_NoGPS");}},false,"","Flight mode when Channel 5 pwm is >=1750"),
    FRAME(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Plus");put(1,"X");put(2,"V");put(3,"H");put(4,"V-Tail");put(5,"A-Tail");put(10,"Y6B-New)");}},false,"","Controls motor mixing for multicopters. Not used for Tri or Traditional Helicopters."),
    FS_BATT_ENABLE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Land");put(2,"RTL");}},false,"","Controls whether failsafe will be invoked when battery voltage or current runs low"),
    FS_BATT_MAH(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.MILLIAMPER_PER_HOUR,false,"","Battery capacity remaining to trigger failsafe. Set to 0 to disable battery remaining failsafe. If the battery remaining drops below this level then the copter will RTL"),
    FS_BATT_VOLTAGE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,14,1,MAV_PARAM_UNIT.VOLT,false,"","Battery voltage to trigger failsafe. Set to 0 to disable battery voltage failsafe. If the battery voltage drops below this voltage then the copter will RTL"),
    FS_GCS_ENABLE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Enabled_always_RTL");put(2,"Enabled_Continue_with_Mission_in_Auto_Mode");}},false,"","Controls whether failsafe will be invoked (MAV_PARAM_GROUP.ARDUCOPTER,and what action to take) when connection with Ground station is lost for at least 5 seconds. NB. The GCS Failsafe is only active when RC_OVERRIDE is being used to control the vehicle."),
    FS_GPS_ENABLE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    FS_THR_ENABLE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Enabled_Always_RTL");put(2,"Enabled_Continue_with_Mission_in_Auto_Mode");put(3,"Enabled_Always_LAND");}},false,"","The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel"),
    FS_THR_VALUE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,975,1,MAV_PARAM_UNIT.PWM,new Range(925,1100),false,"","The PWM level on channel 3 below which throttle failsafe triggers"),
    GND_ABS_PRESS(MAV_PARAM_GROUP_COPTER.GND,50362.25,1,MAV_PARAM_UNIT.PASCAL,true,"","calibrated ground pressure in PASCAL"),
    GND_ALT_OFFSET(MAV_PARAM_GROUP_COPTER.GND,0,1,MAV_PARAM_UNIT.METER,false,"","altitude offset in meters added to barometric altitude. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer. The value is added to the barometric altitude read by the aircraft. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed."),
    GND_TEMP(MAV_PARAM_GROUP_COPTER.GND,29.32966,1,MAV_PARAM_UNIT.DEGREE_CELSIUS,false,"","calibrated ground temperature in degrees Celsius"),
    GPS_HDOP_GOOD(MAV_PARAM_GROUP_COPTER.GPS,230,1,MAV_PARAM_UNIT.UNKNOWN,new Range(100,900),false,"","GPS Hdop value at or below this value represent a good position. Used for pre-arm checks"),
    GPS_NAVFILTER(MAV_PARAM_GROUP_COPTER.GPS,8,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Portable");put(2,"Stationary");put(3,"Pedestrian");put(4,"Automotive");put(5,"Sea");put(6,"Airborne1G");put(7,"Airborne2G");put(8,"Airborne4G");}},false,"","Navigation filter engine setting"),
    GPS_TYPE(MAV_PARAM_GROUP_COPTER.GPS,1,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"None");put(1,"AUTO");put(2,"uBlox");put(3,"MTK");put(4,"MTK19");put(5,"NMEA");put(6,"SiRF");put(7,"HIL");put(8,"SwiftNav");put(9,"PX4-UAVCAN");put(10,"SBF");put(11,"GSOF");put(12,"QURT");put(13,"ERB");put(14,"MAV");put(15,"NOVA");}},false,"","GPS type"),
    GPSGLITCH_ACCEL(MAV_PARAM_GROUP_COPTER.GPS,1000,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    GPSGLITCH_ENABLE(MAV_PARAM_GROUP_COPTER.GPS,1,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    GPSGLITCH_RADIUS(MAV_PARAM_GROUP_COPTER.GPS,200,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    HLD_LAT_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    INAV_TC_XY(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,2.5,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    INAV_TC_Z(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,5,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    INS_ACCOFFS_X(MAV_PARAM_GROUP_COPTER.INS,0.02704384,1,MAV_PARAM_UNIT.METER_PER_SECOND_PER_SECOND,new Range(-3.5,3.5),false,"Accelerometer offsets of X axis","This is setup using the acceleration calibration or level operations"),
    INS_ACCOFFS_Y(MAV_PARAM_GROUP_COPTER.INS,0.07232188,1,MAV_PARAM_UNIT.METER_PER_SECOND_PER_SECOND,new Range(-3.5,3.5),false,"Accelerometer offsets of Y axis","This is setup using the acceleration calibration or level operations"),
    INS_ACCOFFS_Z(MAV_PARAM_GROUP_COPTER.INS,-0.2916607,1,MAV_PARAM_UNIT.METER_PER_SECOND_PER_SECOND,new Range(-3.5,3.5),false,"Accelerometer offsets of Z axis","This is setup using the acceleration calibration or level operations"),
    INS_ACCSCAL_X(MAV_PARAM_GROUP_COPTER.INS,0.9998162,1,MAV_PARAM_UNIT.UNKNOWN,new Range(0.8,1.2),false,"Accelerometer scaling of X axis","Calculated during acceleration calibration routine"),
    INS_ACCSCAL_Y(MAV_PARAM_GROUP_COPTER.INS,0.9920868,1,MAV_PARAM_UNIT.UNKNOWN,new Range(0.8,1.2),false,"Accelerometer scaling of Y axis","Calculated during acceleration calibration routine"),
    INS_ACCSCAL_Z(MAV_PARAM_GROUP_COPTER.INS,1.0001,1,MAV_PARAM_UNIT.UNKNOWN,new Range(0.8,1.2),false,"Accelerometer scaling of Z axis","Calculated during acceleration calibration routine"),
    INS_GYROFFS_X(MAV_PARAM_GROUP_COPTER.INS,-0.003293759,1,MAV_PARAM_UNIT.RADIAN_PER_SECOND,false,"Gyro sensor offsets of X axis","This is setup on each boot during gyro calibrations"),
    INS_GYROFFS_Y(MAV_PARAM_GROUP_COPTER.INS,-0.07851811,1,MAV_PARAM_UNIT.RADIAN_PER_SECOND,false,"Gyro sensor offsets of Y axis","This is setup on each boot during gyro calibrations"),
    INS_GYROFFS_Z(MAV_PARAM_GROUP_COPTER.INS,0.02421099,1,MAV_PARAM_UNIT.RADIAN_PER_SECOND,false,"Gyro sensor offsets of Z axis","This is setup on each boot during gyro calibrations"),
    INS_MPU6K_FILTER(MAV_PARAM_GROUP_COPTER.INS,0,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    INS_PRODUCT_ID(MAV_PARAM_GROUP_COPTER.INS,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Unknown");put(1,"unused");put(2,"unused");put(88,"unused");put(3,"SITL");put(4,"PX4v1");put(5,"PX4v2");put(256,"unused");put(257,"Linux");}},false,"","Which type of IMU is installed (MAV_PARAM_GROUP.ARDUCOPTER,read-only)."),
    LAND_REPOSITION(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"NoRepositioning");put(1,"Repositioning");}},false,"","Enables user input during LAND mode, the landing phase of RTL, and auto mode landings."),
    LAND_SPEED(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,50,1,MAV_PARAM_UNIT.CENTIMETER_PER_SECOND,new Range(30,200),false,"","The descent speed for the final stage of landing in CENTIMETER/s"),
    LOG_BITMASK(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,26622,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(830,"Default");put(894,"Default+RCIN");put(958,"Default+IMU");put(1854,"Default+Motors");put(-6146,"NearlyAll-AC315");put(45054,"NearlyAll");put(131071,"All+FastATT");put(262142,"All+MotBatt");put(393214,"All+FastIMU");put(397310,"All+FastIMU+PID");put(655358,"All+FullIMU");put(0,"Disabled");}},false,"","4 byte bitmap of log types to enable, Bitmask: 0:ATTITUDE_FAST,1:ATTITUDE_MED,2:GPS,3:PM,4:CTUN,5:NTUN,6:RCIN,7:IMU,8:CMD,9:CURRENT,10:RCOUT,11:OPTFLOW,12:PID,13:COMPASS,14:INAV,15:CAMERA,17:MOTBATT,18:IMU_FAST,19:IMU_RAW"),
    LOITER_LAT_D(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    LOITER_LAT_I(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.5,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    LOITER_LAT_IMAX(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1000,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    LOITER_LAT_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    LOITER_LON_D(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    LOITER_LON_I(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.5,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    LOITER_LON_IMAX(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1000,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    LOITER_LON_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    MAG_ENABLE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Enabled");}},false,"","Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass"),
    MIS_RESTART(MAV_PARAM_GROUP_COPTER.MIS,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"ResumeMission");put(1,"RestartMission");}},false,"Mission Restart when entering Auto mode","Controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)"),
    MIS_TOTAL(MAV_PARAM_GROUP_COPTER.MIS,6,1,MAV_PARAM_UNIT.NONE,new Range(0,32766),true,"Total mission commands","The number of mission mission items that has been loaded by the ground station. Do not change this manually."),
    MNT_ANGMAX_PAN(MAV_PARAM_GROUP_COPTER.MNT,4500,1,MAV_PARAM_UNIT.CENTIDEGREE,new Range(-18000,17999),false,"","Maximum physical pan (yaw) angular position of the mount"),
    MNT_ANGMAX_ROL(MAV_PARAM_GROUP_COPTER.MNT,4500,1,MAV_PARAM_UNIT.CENTIDEGREE,new Range(-18000,17999),false,"","Maximum physical roll angular position of the mount"),
    MNT_ANGMAX_TIL(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.CENTIDEGREE,new Range(-18000,17999),false,"","Maximum physical tilt (pitch) angular position of the mount"),
    MNT_ANGMIN_PAN(MAV_PARAM_GROUP_COPTER.MNT,-4500,1,MAV_PARAM_UNIT.CENTIDEGREE,new Range(-18000,17999),false,"","Minimum physical pan (yaw) angular position of mount."),
    MNT_ANGMIN_ROL(MAV_PARAM_GROUP_COPTER.MNT,-4500,1,MAV_PARAM_UNIT.CENTIDEGREE,new Range(-18000,17999),false,"","Minimum physical roll angular position of mount."),
    MNT_ANGMIN_TIL(MAV_PARAM_GROUP_COPTER.MNT,-9000,1,MAV_PARAM_UNIT.CENTIDEGREE,new Range(-18000,17999),false,"","Minimum physical tilt (pitch) angular position of mount."),
    MNT_CONTROL_X(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    MNT_CONTROL_Y(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    MNT_CONTROL_Z(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    MNT_JSTICK_SPD(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.UNKNOWN,new Range(0,100),false,"","0 for position control, small for low speeds, 100 for max speed. A good general value is 10 which gives a movement speed of 3 degrees per second."),
    MNT_MODE(MAV_PARAM_GROUP_COPTER.MNT,3,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    MNT_NEUTRAL_X(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.DEGREE,new Range(-180.00,179.99),false,"","Mount roll angle when in neutral position"),
    MNT_NEUTRAL_Y(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.DEGREE,new Range(-180.00,179.99),false,"","Mount tilt/pitch angle when in neutral position"),
    MNT_NEUTRAL_Z(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.DEGREE,new Range(-180.00,179.99),false,"","Mount pan/yaw angle when in neutral position"),
    MNT_RC_IN_PAN(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(5,"RC5");put(6,"RC6");put(7,"RC7");put(8,"RC8");put(9,"RC9");put(10,"RC10");put(11,"RC11");put(12,"RC12");}},false,"","0 for none, any other for the RC channel to be used to control pan (MAV_PARAM_GROUP.ARDUCOPTER,yaw) movements"),
    MNT_RC_IN_ROLL(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(5,"RC5");put(6,"RC6");put(7,"RC7");put(8,"RC8");put(9,"RC9");put(10,"RC10");put(11,"RC11");put(12,"RC12");}},false,"","0 for none, any other for the RC channel to be used to control roll movements"),
    MNT_RC_IN_TILT(MAV_PARAM_GROUP_COPTER.MNT,6,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(5,"RC5");put(6,"RC6");put(7,"RC7");put(8,"RC8");put(9,"RC9");put(10,"RC10");put(11,"RC11");put(12,"RC12");}},false,"","0 for none, any other for the RC channel to be used to control tilt (MAV_PARAM_GROUP.ARDUCOPTER,pitch) movements"),
    MNT_RETRACT_X(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.DEGREE,new Range(-180.00,179.99),false,"","Mount roll angle when in retracted position"),
    MNT_RETRACT_Y(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.DEGREE,new Range(-180.00,179.99),false,"","Mount tilt/pitch angle when in retracted position"),
    MNT_RETRACT_Z(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.DEGREE,new Range(-180.00,179.99),false,"","Mount yaw/pan angle when in retracted position"),
    MNT_STAB_PAN(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Enabled");}},false,"","enable pan/yaw stabilisation relative to Earth"),
    MNT_STAB_ROLL(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Enabled");}},false,"","enable roll stabilisation relative to Earth"),
    MNT_STAB_TILT(MAV_PARAM_GROUP_COPTER.MNT,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Enabled");}},false,"","enable tilt/pitch stabilisation relative to Earth"),
    MOT_SPIN_ARMED(MAV_PARAM_GROUP_COPTER.MOT,90,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    MOT_TCRV_ENABLE(MAV_PARAM_GROUP_COPTER.MOT,1,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    MOT_TCRV_MAXPCT(MAV_PARAM_GROUP_COPTER.MOT,93,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    MOT_TCRV_MIDPCT(MAV_PARAM_GROUP_COPTER.MOT,52,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    OF_PIT_D(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.12,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    OF_PIT_I(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.5,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    OF_PIT_IMAX(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,100,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    OF_PIT_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,2.5,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    OF_RLL_D(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.12,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    OF_RLL_I(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.5,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    OF_RLL_IMAX(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,100,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    OF_RLL_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,2.5,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    PHLD_BRAKE_ANGLE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,3000,1,MAV_PARAM_UNIT.CENTIDEGREE,new Range(2000,4500),false,"","PosHold flight mode's max lean angle during braking in centi-degrees"),
    PHLD_BRAKE_RATE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,8,1,MAV_PARAM_UNIT.DEGREE_PER_SECOND,new Range(4,12),false,"PosHold braking rate","PosHold flight mode's rotation rate during braking in deg/sec"),
    PILOT_ACCEL_Z(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,250,1,MAV_PARAM_UNIT.CENTIMETER_PER_SECOND_PER_SECOND,new Range(50,500),false,"","The vertical acceleration used when pilot is controlling the altitude"),
    PILOT_VELZ_MAX(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,250,1,MAV_PARAM_UNIT.CENTIMETER_PER_SECOND,new Range(50,500),false,"","The maximum vertical velocity the pilot may request in CENTIMETER/s"),
    POSCON_THR_HOVER(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,724,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RATE_PIT_D(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.0055,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RATE_PIT_I(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.07999999,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RATE_PIT_IMAX(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1000,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RATE_PIT_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.07999999,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RATE_RLL_D(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.003,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RATE_RLL_I(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.08499999,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RATE_RLL_IMAX(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1000,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RATE_RLL_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.08499999,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RATE_YAW_D(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.003,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RATE_YAW_I(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.02,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RATE_YAW_IMAX(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1000,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RATE_YAW_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.17,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RC_FEEL_RP(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,100,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(0,"Standard");put(1000,"VerySoft");put(25,"Soft");put(50,"Medium");put(75,"Crisp");put(100,"VeryCrisp");}},false,"","RC feel for roll/pitch which controls vehicle response to user input with 0 being extremely soft and 100 being crisp"),
    RC_SPEED(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,490,1,MAV_PARAM_UNIT.HERTZ,new Range(50,490),false,"","This is the speed in Hertz that your ESCs will receive updates"),
    RC1_DZ(MAV_PARAM_GROUP_COPTER.RC1,30,1,MAV_PARAM_UNIT.PWM,new Range(0,200),false,"RC dead-zone","dead zone around trim or bottom"),
    RC1_MAX(MAV_PARAM_GROUP_COPTER.RC1,1976,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC max PWM","RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC1_MIN(MAV_PARAM_GROUP_COPTER.RC1,998,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC min PWM","RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC1_REV(MAV_PARAM_GROUP_COPTER.RC1,1,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(-1,"Reversed");put(1,"Normal");}},false,"RC reversed","Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel."),
    RC1_TRIM(MAV_PARAM_GROUP_COPTER.RC1,1483,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"","RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC10_DZ(MAV_PARAM_GROUP_COPTER.RC10,0,1,MAV_PARAM_UNIT.PWM,new Range(0,200),false,"RC dead-zone","dead zone around trim or bottom"),
    RC10_FUNCTION(MAV_PARAM_GROUP_COPTER.RC10,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"RCPassThru");put(2,"Flap");put(3,"Flap_auto");put(4,"Aileron");put(6,"mount_pan");put(7,"mount_tilt");put(8,"mount_roll");put(9,"mount_open");put(10,"camera_trigger");put(11,"release");put(12,"mount2_pan");put(13,"mount2_tilt");put(14,"mount2_roll");put(15,"mount2_open");put(16,"DifferentialSpoiler1");put(17,"DifferentialSpoiler2");put(18,"AileronWithInput");put(19,"Elevator");put(20,"ElevatorWithInput");put(21,"Rudder");put(24,"Flaperon1");put(25,"Flaperon2");put(26,"GroundSteering");put(27,"Parachute");put(28,"EPM");put(29,"LandingGear");put(30,"EngineRunEnable");put(31,"HeliRSC");put(32,"HeliTailRSC");put(33,"Motor1");put(34,"Motor2");put(35,"Motor3");put(36,"Motor4");put(37,"Motor5");put(38,"Motor6");put(39,"Motor7");put(40,"Motor8");put(51,"RCIN1");put(52,"RCIN2");put(53,"RCIN3");put(54,"RCIN4");put(55,"RCIN5");put(56,"RCIN6");put(57,"RCIN7");put(58,"RCIN8");put(59,"RCIN9");put(60,"RCIN10");put(61,"RCIN11");put(62,"RCIN12");put(63,"RCIN13");put(64,"RCIN14");put(65,"RCIN15");put(66,"RCIN16");put(67,"Ignition");put(68,"Choke");put(69,"Starter");put(70,"Throttle");}},false,"","Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function"),
    RC10_MAX(MAV_PARAM_GROUP_COPTER.RC10,1900,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC max PWM","RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC10_MIN(MAV_PARAM_GROUP_COPTER.RC10,1100,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC min PWM","RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC10_REV(MAV_PARAM_GROUP_COPTER.RC10,1,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(-1,"Reversed");put(1,"Normal");}},false,"RC reversed","Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel."),
    RC10_TRIM(MAV_PARAM_GROUP_COPTER.RC10,0,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"","RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC11_DZ(MAV_PARAM_GROUP_COPTER.RC11,0,1,MAV_PARAM_UNIT.PWM,new Range(0,200),false,"","dead zone around trim or bottom"),
    RC11_FUNCTION(MAV_PARAM_GROUP_COPTER.RC11,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"RCPassThru");put(2,"Flap");put(3,"Flap_auto");put(4,"Aileron");put(6,"mount_pan");put(7,"mount_tilt");put(8,"mount_roll");put(9,"mount_open");put(10,"camera_trigger");put(11,"release");put(12,"mount2_pan");put(13,"mount2_tilt");put(14,"mount2_roll");put(15,"mount2_open");put(16,"DifferentialSpoiler1");put(17,"DifferentialSpoiler2");put(18,"AileronWithInput");put(19,"Elevator");put(20,"ElevatorWithInput");put(21,"Rudder");put(24,"Flaperon1");put(25,"Flaperon2");put(26,"GroundSteering");put(27,"Parachute");put(28,"EPM");put(29,"LandingGear");put(30,"EngineRunEnable");put(31,"HeliRSC");put(32,"HeliTailRSC");put(33,"Motor1");put(34,"Motor2");put(35,"Motor3");put(36,"Motor4");put(37,"Motor5");put(38,"Motor6");put(39,"Motor7");put(40,"Motor8");put(51,"RCIN1");put(52,"RCIN2");put(53,"RCIN3");put(54,"RCIN4");put(55,"RCIN5");put(56,"RCIN6");put(57,"RCIN7");put(58,"RCIN8");put(59,"RCIN9");put(60,"RCIN10");put(61,"RCIN11");put(62,"RCIN12");put(63,"RCIN13");put(64,"RCIN14");put(65,"RCIN15");put(66,"RCIN16");put(67,"Ignition");put(68,"Choke");put(69,"Starter");put(70,"Throttle");}},false,"","Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function"),
    RC11_MAX(MAV_PARAM_GROUP_COPTER.RC11,1900,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC max PWM","RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC11_MIN(MAV_PARAM_GROUP_COPTER.RC11,1100,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC min PWM","RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC11_REV(MAV_PARAM_GROUP_COPTER.RC11,1,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(-1,"Reversed");put(1,"Normal");}},false,"RC reversed","Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel."),
    RC11_TRIM(MAV_PARAM_GROUP_COPTER.RC11,0,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"","RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC2_DZ(MAV_PARAM_GROUP_COPTER.RC2,30,1,MAV_PARAM_UNIT.PWM,new Range(0,200),false,"RC dead-zone","dead zone around trim or bottom"),
    RC2_MAX(MAV_PARAM_GROUP_COPTER.RC2,1983,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC max PWM","RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC2_MIN(MAV_PARAM_GROUP_COPTER.RC2,996,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC min PWM","RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC2_REV(MAV_PARAM_GROUP_COPTER.RC2,1,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(-1,"Reversed");put(1,"Normal");}},false,"RC reversed","Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel."),
    RC2_TRIM(MAV_PARAM_GROUP_COPTER.RC2,1492,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"","RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC3_DZ(MAV_PARAM_GROUP_COPTER.RC3,30,1,MAV_PARAM_UNIT.PWM,new Range(0,200),false,"RC dead-zone","dead zone around trim or bottom"),
    RC3_MAX(MAV_PARAM_GROUP_COPTER.RC3,1982,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC max PWM","RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC3_MIN(MAV_PARAM_GROUP_COPTER.RC3,996,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC min PWM","RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC3_REV(MAV_PARAM_GROUP_COPTER.RC3,1,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(-1,"Reversed");put(1,"Normal");}},false,"RC reversed","Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel."),
    RC3_TRIM(MAV_PARAM_GROUP_COPTER.RC3,1000,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"","RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC4_DZ(MAV_PARAM_GROUP_COPTER.RC4,40,1,MAV_PARAM_UNIT.PWM,new Range(0,200),false,"RC dead-zone","dead zone around trim or bottom"),
    RC4_MAX(MAV_PARAM_GROUP_COPTER.RC4,1981,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC max PWM","RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC4_MIN(MAV_PARAM_GROUP_COPTER.RC4,992,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC min PWM","RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC4_REV(MAV_PARAM_GROUP_COPTER.RC4,1,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(-1,"Reversed");put(1,"Normal");}},false,"RC reversed","Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel."),
    RC4_TRIM(MAV_PARAM_GROUP_COPTER.RC4,1486,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"","RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC5_DZ(MAV_PARAM_GROUP_COPTER.RC5,0,1,MAV_PARAM_UNIT.PWM,new Range(0,200),false,"","dead zone around trim or bottom"),
    RC5_FUNCTION(MAV_PARAM_GROUP_COPTER.RC5,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"RCPassThru");put(2,"Flap");put(3,"Flap_auto");put(4,"Aileron");put(6,"mount_pan");put(7,"mount_tilt");put(8,"mount_roll");put(9,"mount_open");put(10,"camera_trigger");put(11,"release");put(12,"mount2_pan");put(13,"mount2_tilt");put(14,"mount2_roll");put(15,"mount2_open");put(16,"DifferentialSpoiler1");put(17,"DifferentialSpoiler2");put(18,"AileronWithInput");put(19,"Elevator");put(20,"ElevatorWithInput");put(21,"Rudder");put(24,"Flaperon1");put(25,"Flaperon2");put(26,"GroundSteering");put(27,"Parachute");put(28,"EPM");put(29,"LandingGear");put(30,"EngineRunEnable");put(31,"HeliRSC");put(32,"HeliTailRSC");put(33,"Motor1");put(34,"Motor2");put(35,"Motor3");put(36,"Motor4");put(37,"Motor5");put(38,"Motor6");put(39,"Motor7");put(40,"Motor8");put(51,"RCIN1");put(52,"RCIN2");put(53,"RCIN3");put(54,"RCIN4");put(55,"RCIN5");put(56,"RCIN6");put(57,"RCIN7");put(58,"RCIN8");put(59,"RCIN9");put(60,"RCIN10");put(61,"RCIN11");put(62,"RCIN12");put(63,"RCIN13");put(64,"RCIN14");put(65,"RCIN15");put(66,"RCIN16");put(67,"Ignition");put(68,"Choke");put(69,"Starter");put(70,"Throttle");}},false,"","Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function"),
    RC5_MAX(MAV_PARAM_GROUP_COPTER.RC5,1982,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC max PWM","RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC5_MIN(MAV_PARAM_GROUP_COPTER.RC5,992,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC min PWM","RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC5_REV(MAV_PARAM_GROUP_COPTER.RC5,1,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(-1,"Reversed");put(1,"Normal");}},false,"RC reversed","Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel."),
    RC5_TRIM(MAV_PARAM_GROUP_COPTER.RC5,993,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"","RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC6_DZ(MAV_PARAM_GROUP_COPTER.RC6,0,1,MAV_PARAM_UNIT.PWM,new Range(0,200),false,"RC dead-zone","dead zone around trim or bottom"),
    RC6_FUNCTION(MAV_PARAM_GROUP_COPTER.RC6,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"RCPassThru");put(2,"Flap");put(3,"Flap_auto");put(4,"Aileron");put(6,"mount_pan");put(7,"mount_tilt");put(8,"mount_roll");put(9,"mount_open");put(10,"camera_trigger");put(11,"release");put(12,"mount2_pan");put(13,"mount2_tilt");put(14,"mount2_roll");put(15,"mount2_open");put(16,"DifferentialSpoiler1");put(17,"DifferentialSpoiler2");put(18,"AileronWithInput");put(19,"Elevator");put(20,"ElevatorWithInput");put(21,"Rudder");put(24,"Flaperon1");put(25,"Flaperon2");put(26,"GroundSteering");put(27,"Parachute");put(28,"EPM");put(29,"LandingGear");put(30,"EngineRunEnable");put(31,"HeliRSC");put(32,"HeliTailRSC");put(33,"Motor1");put(34,"Motor2");put(35,"Motor3");put(36,"Motor4");put(37,"Motor5");put(38,"Motor6");put(39,"Motor7");put(40,"Motor8");put(51,"RCIN1");put(52,"RCIN2");put(53,"RCIN3");put(54,"RCIN4");put(55,"RCIN5");put(56,"RCIN6");put(57,"RCIN7");put(58,"RCIN8");put(59,"RCIN9");put(60,"RCIN10");put(61,"RCIN11");put(62,"RCIN12");put(63,"RCIN13");put(64,"RCIN14");put(65,"RCIN15");put(66,"RCIN16");put(67,"Ignition");put(68,"Choke");put(69,"Starter");put(70,"Throttle");}},false,"","Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function"),
    RC6_MAX(MAV_PARAM_GROUP_COPTER.RC6,1985,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC max PWM","RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC6_MIN(MAV_PARAM_GROUP_COPTER.RC6,992,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC min PWM","RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC6_REV(MAV_PARAM_GROUP_COPTER.RC6,1,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(-1,"Reversed");put(1,"Normal");}},false,"RC reversed","Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel."),
    RC6_TRIM(MAV_PARAM_GROUP_COPTER.RC6,992,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"","RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC7_DZ(MAV_PARAM_GROUP_COPTER.RC7,0,1,MAV_PARAM_UNIT.PWM,new Range(0,200),false,"RC dead-zone","dead zone around trim or bottom"),
    RC7_FUNCTION(MAV_PARAM_GROUP_COPTER.RC7,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"RCPassThru");put(2,"Flap");put(3,"Flap_auto");put(4,"Aileron");put(6,"mount_pan");put(7,"mount_tilt");put(8,"mount_roll");put(9,"mount_open");put(10,"camera_trigger");put(11,"release");put(12,"mount2_pan");put(13,"mount2_tilt");put(14,"mount2_roll");put(15,"mount2_open");put(16,"DifferentialSpoiler1");put(17,"DifferentialSpoiler2");put(18,"AileronWithInput");put(19,"Elevator");put(20,"ElevatorWithInput");put(21,"Rudder");put(24,"Flaperon1");put(25,"Flaperon2");put(26,"GroundSteering");put(27,"Parachute");put(28,"EPM");put(29,"LandingGear");put(30,"EngineRunEnable");put(31,"HeliRSC");put(32,"HeliTailRSC");put(33,"Motor1");put(34,"Motor2");put(35,"Motor3");put(36,"Motor4");put(37,"Motor5");put(38,"Motor6");put(39,"Motor7");put(40,"Motor8");put(51,"RCIN1");put(52,"RCIN2");put(53,"RCIN3");put(54,"RCIN4");put(55,"RCIN5");put(56,"RCIN6");put(57,"RCIN7");put(58,"RCIN8");put(59,"RCIN9");put(60,"RCIN10");put(61,"RCIN11");put(62,"RCIN12");put(63,"RCIN13");put(64,"RCIN14");put(65,"RCIN15");put(66,"RCIN16");put(67,"Ignition");put(68,"Choke");put(69,"Starter");put(70,"Throttle");}},false,"","Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function"),
    RC7_MAX(MAV_PARAM_GROUP_COPTER.RC7,1900,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC max PWM","RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC7_MIN(MAV_PARAM_GROUP_COPTER.RC7,1100,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC min PWM","RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC7_REV(MAV_PARAM_GROUP_COPTER.RC7,1,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(-1,"Reversed");put(1,"Normal");}},false,"RC reversed","Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel."),
    RC7_TRIM(MAV_PARAM_GROUP_COPTER.RC7,1498,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"","RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC8_DZ(MAV_PARAM_GROUP_COPTER.RC8,0,1,MAV_PARAM_UNIT.PWM,new Range(0,200),false,"RC dead-zone","dead zone around trim or bottom"),
    RC8_FUNCTION(MAV_PARAM_GROUP_COPTER.RC8,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"RCPassThru");put(2,"Flap");put(3,"Flap_auto");put(4,"Aileron");put(6,"mount_pan");put(7,"mount_tilt");put(8,"mount_roll");put(9,"mount_open");put(10,"camera_trigger");put(11,"release");put(12,"mount2_pan");put(13,"mount2_tilt");put(14,"mount2_roll");put(15,"mount2_open");put(16,"DifferentialSpoiler1");put(17,"DifferentialSpoiler2");put(18,"AileronWithInput");put(19,"Elevator");put(20,"ElevatorWithInput");put(21,"Rudder");put(24,"Flaperon1");put(25,"Flaperon2");put(26,"GroundSteering");put(27,"Parachute");put(28,"EPM");put(29,"LandingGear");put(30,"EngineRunEnable");put(31,"HeliRSC");put(32,"HeliTailRSC");put(33,"Motor1");put(34,"Motor2");put(35,"Motor3");put(36,"Motor4");put(37,"Motor5");put(38,"Motor6");put(39,"Motor7");put(40,"Motor8");put(51,"RCIN1");put(52,"RCIN2");put(53,"RCIN3");put(54,"RCIN4");put(55,"RCIN5");put(56,"RCIN6");put(57,"RCIN7");put(58,"RCIN8");put(59,"RCIN9");put(60,"RCIN10");put(61,"RCIN11");put(62,"RCIN12");put(63,"RCIN13");put(64,"RCIN14");put(65,"RCIN15");put(66,"RCIN16");put(67,"Ignition");put(68,"Choke");put(69,"Starter");put(70,"Throttle");}},false,"","Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function"),
    RC8_MAX(MAV_PARAM_GROUP_COPTER.RC8,1900,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC max PWM","RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC8_MIN(MAV_PARAM_GROUP_COPTER.RC8,1100,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"RC min PWM","RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RC8_REV(MAV_PARAM_GROUP_COPTER.RC8,1,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(-1,"Reversed");put(1,"Normal");}},false,"RC reversed","Reverse servo operation. Set to 1 for normal (MAV_PARAM_GROUP.ARDUCOPTER,forward) operation. Set to -1 to reverse this channel."),
    RC8_TRIM(MAV_PARAM_GROUP_COPTER.RC8,1498,1,MAV_PARAM_UNIT.PWM,new Range(800,2200),false,"","RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit."),
    RCMAP_PITCH(MAV_PARAM_GROUP_COPTER.RCMAP,2,1,MAV_PARAM_UNIT.NONE,false,"","Pitch channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Pitch is normally on channel 2, but you can move it to any channel with this parameter. Reboot is required for changes to take effect."),
    RCMAP_ROLL(MAV_PARAM_GROUP_COPTER.RCMAP,1,1,MAV_PARAM_UNIT.NONE,false,"","Roll channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Roll is normally on channel 1, but you can move it to any channel with this parameter. Reboot is required for changes to take effect."),
    RCMAP_THROTTLE(MAV_PARAM_GROUP_COPTER.RCMAP,3,1,MAV_PARAM_UNIT.NONE,false,"","Throttle channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Throttle is normally on channel 3, but you can move it to any channel with this parameter. Warning APM 2.X: Changing the throttle channel could produce unexpected fail-safe results if connection between receiver and on-board PPM Encoder is lost. Disabling on-board PPM Encoder is recommended. Reboot is required for changes to take effect."),
    RCMAP_YAW(MAV_PARAM_GROUP_COPTER.RCMAP,4,1,MAV_PARAM_UNIT.NONE,false,"","Yaw channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Yaw (also known as rudder) is normally on channel 4, but you can move it to any channel with this parameter. Reboot is required for changes to take effect."),
    RELAY_PIN(MAV_PARAM_GROUP_COPTER.RELAY,13,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(-1,"Disabled");put(13,"APM2 A9 pin");put(47,"APM1 relay");put(50,"Pixhawk AUXOUT1");put(51,"Pixhawk AUXOUT2");put(52,"Pixhawk AUXOUT3");put(53,"Pixhawk AUXOUT4");put(54,"Pixhawk AUXOUT5");put(55,"Pixhawk AUXOUT6");put(111,"PX4 FMU Relay1");put(112,"PX4 FMU Relay2");put(113,"PX4IO Relay1");put(114,"PX4IO Relay2");put(115,"PX4IO ACC1");put(116,"PX4IO ACC2");}},false,"","Digital pin number for first relay control. This is the pin used for camera control."),
    RELAY_PIN2(MAV_PARAM_GROUP_COPTER.RELAY,-1,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(-1,"Disabled");put(13,"APM2 A9 pin"); put(47,"APM1 relay");put(50,"Pixhawk AUXOUT1");put(51,"Pixhawk AUXOUT2");put(52,"Pixhawk AUXOUT3");put(53,"Pixhawk AUXOUT4");put(54,"Pixhawk AUXOUT5");put(55,"Pixhawk AUXOUT6");put(111,"PX4 FMU Relay1");put(112,"PX4 FMU Relay2");put(113,"PX4IO Relay1");put(114,"PX4IO Relay2");put(115,"PX4IO ACC1");put(116,"PX4IO ACC2");}},false,"","Digital pin number for 2nd relay control."),
    RNGFND_FUNCTION(MAV_PARAM_GROUP_COPTER.RNGFND,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Linear");put(1,"Inverted");put(2,"Hyperbolic");}},false,"","Control over what function is used to calculate distance. For a linear function, the distance is (MAV_PARAM_GROUP.ARDUCOPTER,voltage-offset)*scaling. For a inverted function the distance is (MAV_PARAM_GROUP.ARDUCOPTER,offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(MAV_PARAM_GROUP.ARDUCOPTER,voltage-offset). The functions return the distance in meters."),
    RNGFND_GAIN(MAV_PARAM_GROUP_COPTER.RNGFND,0.8,1,MAV_PARAM_UNIT.UNKNOWN,new Range(0.01,2.0),false,"","Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter"),
    RNGFND_MAX_CM(MAV_PARAM_GROUP_COPTER.RNGFND,700,1,MAV_PARAM_UNIT.CENTIMETER,false,"","Maximum distance in centimeters that rangefinder can reliably read"),
    RNGFND_MIN_CM(MAV_PARAM_GROUP_COPTER.RNGFND,20,1,MAV_PARAM_UNIT.CENTIMETER,false,"","Minimum distance in centimeters that rangefinder can reliably read"),
    RNGFND_OFFSET(MAV_PARAM_GROUP_COPTER.RNGFND,0,1,MAV_PARAM_UNIT.VOLT,false,"","Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM and I2C Lidars"),
    RNGFND_PIN(MAV_PARAM_GROUP_COPTER.RNGFND,-1,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(-1,"NotUsed");put(0,"APM2-A0");put(1,"APM2-A1");put(2,"APM2-A2");put(3,"APM2-A3");put(4,"APM2-A4");put(5,"APM2-A5");put(6,"APM2-A6");put(7,"APM2-A7");put(8,"APM2-A8");put(9,"APM2-A9");put(11,"PX4-airspeed port");put(15,"Pixhawk-airspeed port");put(64,"APM1-airspeed port");}},false,"","Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port."),
    RNGFND_RMETRIC(MAV_PARAM_GROUP_COPTER.RNGFND,1,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"No");put(1,"Yes");}},false,"","This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (MAV_PARAM_GROUP.ARDUCOPTER,such as the SF/02) have their own internal voltage regulators so they are not ratiometric."),
    RNGFND_SCALING(MAV_PARAM_GROUP_COPTER.RNGFND,3,0.001,MAV_PARAM_UNIT.METER_PER_VOLT,false,"Rangefinder scaling","Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts."),
    RNGFND_SETTLE_MS(MAV_PARAM_GROUP_COPTER.RNGFND,0,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RNGFND_STOP_PIN(MAV_PARAM_GROUP_COPTER.RNGFND,-1,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(-1,"NotUsed");put(50,"Pixhawk AUXOUT1");put(51,"Pixhawk AUXOUT2");put(52,"Pixhawk AUXOUT3,53:Pixhawk,AUXOUT4,54:Pixhawk AUXOUT5");put(55,"Pixhawk AUXOUT6");put(111,"PX4 FMU Relay1");put(112,"PX4 FMU Relay2");put(113,"PX4IO,Relay1");put(114,"PX4IO Relay2");put(115,"PX4IO ACC1");put(116,"PX4IO ACC2");}},false,"","Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other."),
    RNGFND_TYPE(MAV_PARAM_GROUP_COPTER.RNGFND,0,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"None");put(1,"Analog");put(2,"APM2-MaxbotixI2C");put(3,"APM2-PulsedLightI2C");put(4,"PX4-I2C");put(5,"PX4-PWM");put(6,"BBB-PRU");put(7,"LightWareI2C");put(8,"LightWareSerial");put(9,"Bebop");put(10,"MAVLink");put(12,"LeddarOne");}},false,"","What type of rangefinder device that is connected"),
    RSSI_PIN(MAV_PARAM_GROUP_COPTER.RSSI,-1,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RSSI_RANGE(MAV_PARAM_GROUP_COPTER.RSSI, 5,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    RTL_ALT(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1500,1,MAV_PARAM_UNIT.CENTIMETER,new Range(0,8000),false,"","The minimum relative altitude the model will move to before Returning to Launch. Set to zero to return at current altitude."),
    RTL_ALT_FINAL(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.CENTIMETER,new Range(-1,1000),false,"","This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission. Set to zero to land."),
    RTL_LOIT_TIME(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,5000,1,MAV_PARAM_UNIT.METER_PER_SECOND,new Range(0,60000),false,"","Time (in milliseconds) to loiter above home before beginning final descent"),
    SCHED_DEBUG(MAV_PARAM_GROUP_COPTER.SCHED,0,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(0,"Disabled");put(2,"ShowSlips");put(3,"ShowOverruns");}},false,"Scheduler debug level","Set to non-zero to enable scheduler debug messages. When set to show \"Slips\" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table."),
    SERIAL0_BAUD(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,115,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(1,"1200");put(2,"2400");put(4,"4800");put(9,"9600");put(19,"19200");put(38,"38400");put(57,"57600");put(111,"111100");put(115,"115200");put(500,"500000");put(921,"921600");put(1500,"1500000");}},false,"","The baud rate used on the USB console. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults."),
    SERIAL1_BAUD(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,57,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(1,"1200");put(2,"2400");put(4,"4800");put(9,"9600");put(19,"19200");put(38,"38400");put(57,"57600");put(111,"111100");put(115,"115200");put(500,"500000");put(921,"921600");put(1500,"1500000");}},false,"","The baud rate used on the Telem1 port. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults."),
    SIMPLE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,18,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode)"),
    SR0_EXT_STAT(MAV_PARAM_GROUP_COPTER.SR0,2,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station"),
    SR0_EXTRA1(MAV_PARAM_GROUP_COPTER.SR0,4,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station"),
    SR0_EXTRA2(MAV_PARAM_GROUP_COPTER.SR0,4,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of VFR_HUD to ground station"),
    SR0_EXTRA3(MAV_PARAM_GROUP_COPTER.SR0,2,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station"),
    SR0_PARAMS(MAV_PARAM_GROUP_COPTER.SR0,10,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of PARAM_VALUE to ground station"),
    SR0_POSITION(MAV_PARAM_GROUP_COPTER.SR0,2,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of GLOBAL_POSITION_INT to ground station"),
    SR0_RAW_CTRL(MAV_PARAM_GROUP_COPTER.SR0,1,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station"),
    SR0_RAW_SENS(MAV_PARAM_GROUP_COPTER.SR0,2,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station"),
    SR0_RC_CHAN(MAV_PARAM_GROUP_COPTER.SR0,2,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station"),
    SR1_EXT_STAT(MAV_PARAM_GROUP_COPTER.SR1,2,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station"),
    SR1_EXTRA1(MAV_PARAM_GROUP_COPTER.SR1,2,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station"),
    SR1_EXTRA2(MAV_PARAM_GROUP_COPTER.SR1,2,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of VFR_HUD to ground station"),
    SR1_EXTRA3(MAV_PARAM_GROUP_COPTER.SR1,2,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station"),
    SR1_PARAMS(MAV_PARAM_GROUP_COPTER.SR1,0,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of PARAM_VALUE to ground station"),
    SR1_POSITION(MAV_PARAM_GROUP_COPTER.SR1,2,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of GLOBAL_POSITION_INT to ground station"),
    SR1_RAW_CTRL(MAV_PARAM_GROUP_COPTER.SR1,2,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station"),
    SR1_RAW_SENS(MAV_PARAM_GROUP_COPTER.SR1,2,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station"),
    SR1_RC_CHAN(MAV_PARAM_GROUP_COPTER.SR1,2,1,MAV_PARAM_UNIT.HERTZ,new Range(0,10),false,"","Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station"),
    STB_PIT_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.016,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    STB_RLL_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.016,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    STB_YAW_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,4,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    SUPER_SIMPLE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,5,1,MAV_PARAM_UNIT.FLAGS, new HashMap<Integer, String>(){{put(0,"Disabled");put(1,"Mode1");put(2,"Mode2");put(3,"Mode1+2");put(4,"Mode3");put(5,"Mode1+3");put(6,"Mode2+3");put(7,"Mode1+2+3");put(8,"Mode4");put(9,"Mode1+4");put(10,"Mode2+4");put(11,"Mode1+2+4");put(12,"Mode3+4");put(13,"Mode1+3+4");put(14,"Mode2+3+4");put(15,"Mode1+2+3+4");put(16,"Mode5");put(17,"Mode1+5");put(18,"Mode2+5");put(19,"Mode1+2+5");put(20,"Mode3+5");put(21,"Mode1+3+5");put(22,"Mode2+3+5");put(23,"Mode1+2+3+5");put(24,"Mode4+5");put(25,"Mode1+4+5");put(26,"Mode2+4+5");put(27,"Mode1+2+4+5");put(28,"Mode3+4+5");put(29,"Mode1+3+4+5");put(30,"Mode2+3+4+5");put(31,"Mode1+2+3+4+5");put(32,"Mode6");put(33,"Mode1+6");put(34,"Mode2+6");put(35,"Mode1+2+6");put(36,"Mode3+6");put(37,"Mode1+3+6");put(38,"Mode2+3+6");put(39,"Mode1+2+3+6");put(40,"Mode4+6");put(41,"Mode1+4+6");put(42,"Mode2+4+6");put(43,"Mode1+2+4+6");put(44,"Mode3+4+6");put(45,"Mode1+3+4+6");put(46,"Mode2+3+4+6");put(47,"Mode1+2+3+4+6");put(48,"Mode5+6");put(49,"Mode1+5+6");put(50,"Mode2+5+6");put(51,"Mode1+2+5+6");put(52,"Mode3+5+6");put(53,"Mode1+3+5+6");put(54,"Mode2+3+5+6");put(55,"Mode1+2+3+5+6");put(56,"Mode4+5+6");put(57,"Mode1+4+5+6");put(58,"Mode2+4+5+6");put(59,"Mode1+2+4+5+6");put(60,"Mode3+4+5+6");put(61,"Mode1+3+4+5+6");put(62,"Mode2+3+4+5+6");put(63,"Mode1+2+3+4+5+6");}},false,"","Bitmask to enable Super Simple mode for some flight modes. Setting this to Disabled(MAV_PARAM_GROUP.ARDUCOPTER,0) will disable Super Simple Mode"),
    SYSID_MYGCS(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,255,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(255,"Mission Planner and DroidPlanner");put(252,"AP Planner 2");}},false,"My ground station number","Allows restricting radio overrides to only come from my ground station"),
    SYSID_SW_MREV(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,120,1,MAV_PARAM_UNIT.UNKNOWN,true,"Eeprom format version number","This value is incremented when changes are made to the eeprom format"),
    SYSID_SW_TYPE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,10,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(0,"ArduPlane");put(4,"AntennaTracker");put(10,"Copter");put(20,"Rover");}},false,"","This is used by the ground station to recognise the software type (MAV_PARAM_GROUP.ARDUCOPTER,eg ArduPlane vs ArduCopter)"),
    SYSID_THISMAV(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,1,MAV_PARAM_UNIT.UNKNOWN,new Range(1,255),false,"","Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network"),
    TELEM_DELAY(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.SECOND,new Range(0,30),false,"","The amount of time (MAV_PARAM_GROUP.ARDUCOPTER,in seconds) to delay radio telemetry to prevent an Xbee bricking on power up"),
    THR_ACCEL_D(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    THR_ACCEL_I(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    THR_ACCEL_IMAX(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,800,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    THR_ACCEL_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0.5,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    THR_ALT_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    THR_DZ(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,100,1,MAV_PARAM_UNIT.PWM,new Range(0,300),false,"","The deadzone above and below mid throttle. Used in AltHold, Loiter, PosHold flight modes"),
    THR_MAX(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1000,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    THR_MID(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,510,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    THR_MIN(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,130,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    THR_RATE_P(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,6,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    TRIM_THROTTLE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,724,1,MAV_PARAM_UNIT.UNKNOWN,false,"","Coming soon"),
    TUNE(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,1,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(0,"None");put(1,"Stab_Roll/Pitch_kP");put(4,"Rate_Roll/Pitch_kP");put(5,"Rate_Roll/Pitch_kI");put(21,"Rate_Roll/Pitch_kD");put(3,"Stab_Yaw_kP");put(6,"Rate_Yaw_kP");put(26,"Rate_Yaw_kD");put(14,"Altitude_Hold_kP");put(7,"Throttle_Rate_kP");put(34,"Throttle_Accel_kP");put(35,"Throttle_Accel_kI");put(36,"Throttle_Accel_kD");put(42,"Loiter_Speed");put(12,"Loiter_Pos_kP");put(22,"Velocity_XY_kP");put(28,"Velocity_XY_kI");put(10,"WP_Speed");put(25,"Acro_RollPitch_kP");put(40,"Acro_Yaw_kP");put(13,"Heli_Ext_Gyro");put(17,"OF_Loiter_kP");put(18,"OF_Loiter_kI");put(19,"OF_Loiter_kD");put(38,"Declination");put(39,"Circle_Rate");put(41,"RangeFinder_Gain");put(46,"Rate_Pitch_kP");put(47,"Rate_Pitch_kI");put(48,"Rate_Pitch_kD");put(49,"Rate_Roll_kP");put(50,"Rate_Roll_kI");put(51,"Rate_Roll_kD");put(52,"Rate_Pitch_FF");put(53,"Rate_Roll_FF");put(54,"Rate_Yaw_FF");}},false,"","Controls which parameters (MAV_PARAM_GROUP.ARDUCOPTER,normally PID gains) are being tuned with transmitter's channel 6 knob"),
    TUNE_HIGH(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,32,1,MAV_PARAM_UNIT.UNKNOWN,new Range(0,32767),false,"","The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob"),
    TUNE_LOW(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,0,1,MAV_PARAM_UNIT.UNKNOWN,new Range(0,32767),false,"","The minimum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob"),
    WP_YAW_BEHAVIOR(MAV_PARAM_GROUP_COPTER.ARDUCOPTER,2,1,MAV_PARAM_UNIT.FLAGS,new HashMap<Integer, String>(){{put(0,"NeverChangeYaw");put(1,"FaceNextWaypoint");put(2,"FaceNextWaypointExceptRTL");put(3,"FaceAlongGPScourse");}},false,"","Determines how the autopilot controls the yaw during missions and RTL"),
    WPNAV_ACCEL(MAV_PARAM_GROUP_COPTER.WPNAV,100,1,MAV_PARAM_UNIT.CENTIMETER_PER_SECOND_PER_SECOND,new Range(50,500),false,"","Defines the horizontal acceleration in CENTIMETER/s/s used during missions"),
    WPNAV_ACCEL_Z(MAV_PARAM_GROUP_COPTER.WPNAV,100,1,MAV_PARAM_UNIT.CENTIMETER_PER_SECOND_PER_SECOND,new Range(50,500),false,"","Defines the vertical acceleration in CENTIMETER/s/s used during missions"),
    WPNAV_LOIT_JERK(MAV_PARAM_GROUP_COPTER.WPNAV,1000,1,MAV_PARAM_UNIT.CENTIMETER_PER_SECOND_PER_SECOND_PER_SECOND,new Range(500,5000),false,"","Loiter maximum jerk in CENTIMETER/s/s/s"),
    WPNAV_LOIT_SPEED(MAV_PARAM_GROUP_COPTER.WPNAV,1000,1,MAV_PARAM_UNIT.CENTIMETER_PER_SECOND,new Range(20,2000),false,"","Defines the maximum speed in CENTIMETER/s which the aircraft will travel horizontally while in loiter mode"),
    WPNAV_RADIUS(MAV_PARAM_GROUP_COPTER.WPNAV,200,1,MAV_PARAM_UNIT.CENTIMETER,new Range(100,1000),false,"","Defines the distance from a waypoint, that when crossed indicates the wp has been hit."),
    WPNAV_SPEED(MAV_PARAM_GROUP_COPTER.WPNAV,500,1,MAV_PARAM_UNIT.CENTIMETER_PER_SECOND,new Range(0,2000),false,"","Defines the speed in CENTIMETER/s which the aircraft will attempt to maintain horizontally during a WP mission"),
    WPNAV_SPEED_DN(MAV_PARAM_GROUP_COPTER.WPNAV,150,1,MAV_PARAM_UNIT.CENTIMETER_PER_SECOND,new Range(10,500),false,"","Defines the speed in CENTIMETER/s which the aircraft will attempt to maintain while descending during a WP mission"),
    WPNAV_SPEED_UP(MAV_PARAM_GROUP_COPTER.WPNAV,250,1,MAV_PARAM_UNIT.CENTIMETER_PER_SECOND,new Range(10,1000),false,"Waypoint Climb Speed Target","Defines the speed in CENTIMETER/s which the aircraft will attempt to maintain while climbing during a WP mission"),

    ;

    private final Number defaultValue;
    private final Number increment;
    private final MAV_PARAM_UNIT unit;
    private final Range range;
    private final boolean readOnly;
    private final String title;
    private final String description;
    private final Map<Integer, String> options;
    private final MAV_PARAM_GROUP_I group;

    MAV_PARAM_COPTER(MAV_PARAM_GROUP_I group, Number defaultValue, int increment, MAV_PARAM_UNIT unit, Map<Integer, String> options, boolean readOnly, String title, String description) {
        this.group = group;
        this.defaultValue = defaultValue;
        this.increment = increment;
        this.unit = MAV_PARAM_UNIT.FLAGS;
        this.options = options;
        this.range = null;
        this.readOnly = readOnly;
        this.title = title;
        this.description = description;
    }

    MAV_PARAM_COPTER(MAV_PARAM_GROUP_I group, Number defaultValue, Number increment, MAV_PARAM_UNIT unit, Range range, boolean readOnly, String title, String description) {
        this.group = group;
        this.defaultValue = defaultValue;
        this.increment = increment;
        this.unit = unit;
        this.options = null;
        this.range = range;
        this.readOnly = readOnly;
        this.title = title;
        this.description = description;
    }

    MAV_PARAM_COPTER(MAV_PARAM_GROUP_I group, Number defaultValue, Number increment, MAV_PARAM_UNIT unit, boolean readOnly, String title, String description) {
        this.group = group;
        this.defaultValue = defaultValue;
        this.increment = increment;
        this.unit = unit;
        this.options = null;
        this.range = null;
        this.readOnly = readOnly;
        this.title = title;
        this.description = description;
    }

    @Override
    public String toString() {

        return "MAV_PARAMETERS_LIST{" +
                "name=" + this.name() +
                ", defaultValue=" + defaultValue +
                ", increment=" + increment +
                ", unit=" + unit +
                ", readOnly=" + readOnly +
                ", title='" + title + '\'' +
                '}';
    }

    @Override
    public MAV_PARAM_GROUP_I getGroup() {
        return group;
    }

    @Override
    public String getName() {
        return this.name();
    }

    @Override
    public Number getDefaultValue() {
        return defaultValue;
    }

    @Override
    public Number getIncrement() {
        return increment;
    }

    @Override
    public MAV_PARAM_UNIT getUnit() {
        return unit;
    }

    @Override
    public Range getRange() {
        return range;
    }

    @Override
    public Map<Integer, String> getOptions() {
        return options;
    }

    @Override
    public boolean isReadOnly() {
        return readOnly;
    }

    @Override
    public String getTitle() {
        return title;
    }

    @Override
    public String getDescription() {
        return description;
    }

}
