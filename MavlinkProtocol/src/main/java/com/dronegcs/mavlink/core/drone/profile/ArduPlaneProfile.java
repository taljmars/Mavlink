package com.dronegcs.mavlink.core.drone.profile;

import com.dronegcs.mavlink.is.drone.profiles.VehicleProfile;

public class ArduPlaneProfile extends VehicleProfile {

	public ArduPlaneProfile() {
		super.getDefault().setMaxAltitude(100);
	}

	@Override
	public String getParametersDetailsFilePath() {
		return "/com/dronegcs/mavlink/MavlinkParamsPlane.csv";
	}
}
