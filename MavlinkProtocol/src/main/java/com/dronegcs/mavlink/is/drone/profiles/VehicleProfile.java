package com.dronegcs.mavlink.is.drone.profiles;

import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.Map;

public abstract class VehicleProfile {

	private final org.slf4j.Logger LOGGER = LoggerFactory.getLogger(VehicleProfile.class);

	private Map<String, ParameterDetail> parametersMetadata;
	//	private String parameterMetadataType;
	private Default default_ = new Default();

	protected abstract String getParametersDetailsFilePath();

//	public String getParameterMetadataType() {
//		return parameterMetadataType;
//	}

	public Default getDefault() {
		return default_;
	}

	public void setDefault(Default default_) {
		this.default_ = default_;
	}

//	public void setParameterMetadataType(String parameterMetadataType) {
//		this.parameterMetadataType = parameterMetadataType;
//	}

	public void init() {
		try {
			ParameterDetailsParser parameterDetailsParser = new ParameterDetailsParser();
			parametersMetadata = parameterDetailsParser.parse(getParametersDetailsFilePath());
		}
		catch (IOException e) {
			LOGGER.error("Failed buildParser configuration file", e);
			System.err.println("Failed buildParser configuration file '" + e.getMessage() + "'");
		}
	}

	public Map<String, ParameterDetail> getParametersMetadata() {
		return parametersMetadata;
	}

	public static class Default {
		private int wpNavSpeed;
		private int maxAltitude;

		public int getWpNavSpeed() {
			return wpNavSpeed;
		}

		public void setWpNavSpeed(int wpNavSpeed) {
			this.wpNavSpeed = wpNavSpeed;
		}

		public int getMaxAltitude() {
			return maxAltitude;
		}

		public void setMaxAltitude(int maxAltitude) {
			this.maxAltitude = maxAltitude;
		}
	}
}
