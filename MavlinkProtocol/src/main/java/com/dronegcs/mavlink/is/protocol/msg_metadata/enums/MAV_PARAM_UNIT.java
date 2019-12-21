package com.dronegcs.mavlink.is.protocol.msg_metadata.enums;

public enum MAV_PARAM_UNIT {
//    I43475,
//    I43473,
//    Amps_Volt,

    // Fields,
    MILLIGAUSS("milligauss"),
    MILLIGAUSS_PER_AMPER("milligauss/amper"),

    // Power,
    VOLT("volt"),
    PWM("PWM"),
    MILLIAMPER_PER_HOUR("mAh"),

    // Size,
    CENTIMETER("cm"),
    METER("meter"),
    RADIANS("radian"),
    PASCAL("pascal"),
    DEGREE("degree"),
    CENTIDEGREE("centidegree"),

    // Time,
    SECOND("second"),

    // Temp
    DEGREE_CELSIUS("degrees-celsius"),

    // Speed,
    HERTZ("Hz"),
    RADIAN_PER_SECOND("rad/s"),
    DEGREE_PER_SECOND("deg/s"),
    METER_PER_SECOND("m/s"),
    METER_PER_SECOND_PER_SECOND("m/s/s"),
    CENTIMETER_PER_SECOND("cm/s"),
    CENTIMETER_PER_SECOND_PER_SECOND("cm/s/s"),
    CENTIMETER_PER_SECOND_PER_SECOND_PER_SECOND("cm/s/s/s"),
    CENTIDEGREE_PER_SECOND("centidegrees/s"),
    CENTIDEGREE_PER_SECOND_PER_SECOND("centidegrees/s/s"),
//    meters_Volt,

    // General
    PERCENT("percent"),
    UNKNOWN("unknown"),
    NONE(""),
    FLAGS("list"),
    ;

    private final String name;

    MAV_PARAM_UNIT(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return name;
    }
}
