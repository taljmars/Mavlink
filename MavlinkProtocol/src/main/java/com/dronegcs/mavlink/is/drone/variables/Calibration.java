package com.dronegcs.mavlink.is.drone.variables;


import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.msg_command_long;
import org.springframework.stereotype.Component;

import com.dronegcs.mavlink.is.drone.DroneVariable;
import com.dronegcs.mavlink.is.drone.DroneInterfaces.DroneEventsType;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.msg_statustext;
import com.dronegcs.mavlink.is.protocol.msgbuilder.MavLinkCalibration;

@Component
public class Calibration extends DroneVariable {

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

	public void sendAck(int step) {
		MavLinkCalibration.sendAccelCalibrationAckMessage(step, drone);
	}

	public void processMessage(MAVLinkMessage msg) {
		if (msg.msgid == msg_statustext.MAVLINK_MSG_ID_STATUSTEXT) {
			msg_statustext statusMsg = (msg_statustext) msg;
			mavMsg = statusMsg.getText();

			if (mavMsg.contains("Calibration"))
				calibrating = false;

			drone.notifyDroneEvent(DroneEventsType.CALIBRATION_IMU);
		}
	}

	public String getMessage() {
		return mavMsg;
	}

	public void setCalibrating(boolean flag) {
		calibrating = flag;
	}

	public boolean isCalibrating() {
		return calibrating;
	}
}
