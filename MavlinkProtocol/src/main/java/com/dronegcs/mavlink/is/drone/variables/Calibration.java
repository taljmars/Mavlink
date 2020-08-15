package com.dronegcs.mavlink.is.drone.variables;


import com.dronegcs.mavlink.is.drone.Drone;
import com.dronegcs.mavlink.is.drone.DroneInterfaces;
import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.msg_command_long;
import org.springframework.stereotype.Component;

import com.dronegcs.mavlink.is.drone.DroneVariable;
import com.dronegcs.mavlink.is.drone.DroneInterfaces.DroneEventsType;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.msg_statustext;
import com.dronegcs.mavlink.is.protocol.msgbuilder.MavLinkCalibration;

@Component
public class Calibration extends DroneVariable implements DroneInterfaces.OnDroneListener {

	private String mavMsg;
	private boolean calibrating;

	public boolean startAccelerometerCalibration() {
        if(drone.getState().isFlying()) {
            calibrating = false;
        }
        else {
            calibrating = true;
            MavLinkCalibration.sendStartAccelCalibrationMessage(drone);
        }
        return calibrating;
	}

	public boolean startLevelCalibration() {
		if(drone.getState().isFlying()) {
			return false;
		}

		MavLinkCalibration.sendStartLevelCalibrationMessage(drone);
		return true;
	}

	public boolean startMagnometerCalibration() {
		if(drone.getState().isFlying()) {
			calibrating = false;
		}
		else {
			calibrating = true;
		}

		MavLinkCalibration.sendStartMagnometerCalibrationMessage(drone);
		return calibrating;
	}

	public void sendAccelCalibrationAck(int step) {
		MavLinkCalibration.sendAccelCalibrationAckMessage(step, drone);
	}

	@Override
	public void onDroneEvent(DroneEventsType event, Drone drone) {
		switch (event) {
			case MAGNETOMETER:
				break;

			case CALIBRATION_IMU:
				break;

			default:
				break;
		}
	}

	public String getMessage() {
		return mavMsg;
	}

	public void setCalibrating(boolean flag) {
		calibrating = flag;
		drone.notifyDroneEvent(DroneEventsType.CALIBRATION_IMU);
	}

	public boolean isCalibrating() {
		return calibrating;
	}
}
