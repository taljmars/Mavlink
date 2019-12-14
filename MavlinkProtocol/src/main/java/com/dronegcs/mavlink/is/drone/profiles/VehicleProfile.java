package com.dronegcs.mavlink.is.drone.profiles;

import org.springframework.core.io.ClassPathResource;

import java.io.File;
import java.util.Map;

public abstract class VehicleProfile {
	private final Map<String, ParameterDetail> parametersMetadata;
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

	public VehicleProfile() {
		ParameterDetailsParser parameterDetailsParser = new ParameterDetailsParser();
		parametersMetadata = parameterDetailsParser.parse(getParametersDetailsFilePath());
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
