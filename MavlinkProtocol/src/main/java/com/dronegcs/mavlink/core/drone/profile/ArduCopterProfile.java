package com.dronegcs.mavlink.core.drone.profile;

import com.dronegcs.mavlink.is.drone.profiles.VehicleProfile;

public class ArduCopterProfile extends VehicleProfile {
	
	public ArduCopterProfile() {
		super.getDefault().setMaxAltitude(100);
		super.getDefault().setWpNavSpeed(3);
	}

	@Override
	public String getParametersDetailsFilePath() {
		return "/com/dronegcs/mavlink/MavlinkParamsCopter.csv";
	}
}
