package com.dronegcs.mavlink.core.firmware;

public enum FirmwareType {
	ARDU_PLANE("ArduPlane"),
	ARDU_COPTER("ArduCopter"),
	ARDU_COPTER2("ArduCopter2"),

	UNKNOWN("Unknown");

	private final String type;

	FirmwareType(String type) {
		this.type = type;
	}

	@Override
	public String toString() {
		return type;
	}

	public static FirmwareType firmwareFromString(String str) {
		if (str.equalsIgnoreCase(ARDU_PLANE.type)) {
			return ARDU_PLANE;
		}

		if (str.equalsIgnoreCase(ARDU_COPTER.type)) {
			return ARDU_COPTER;
		}

		if (str.equalsIgnoreCase(ARDU_COPTER2.type)) {
			return ARDU_COPTER2;
		}

		return UNKNOWN;
	}
}
