# ArduPilot Parameters

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
