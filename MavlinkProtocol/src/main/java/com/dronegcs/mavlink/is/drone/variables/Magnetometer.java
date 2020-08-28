package com.dronegcs.mavlink.is.drone.variables;


import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.msg_scaled_imu;
import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.msg_scaled_imu2;
import org.springframework.stereotype.Component;

import com.dronegcs.mavlink.is.drone.DroneVariable;
import com.dronegcs.mavlink.is.drone.DroneInterfaces.DroneEventsType;
import com.dronegcs.mavlink.is.drone.parameters.Parameter;
import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.msg_raw_imu;

@Component
public class Magnetometer extends DroneVariable {

	private int xmag;
	private int ymag;
	private int zmag;
	private int xacc;
	private int yacc;
	private int zacc;
	private int xgyro;
	private int ygyro;
	private int zgyro;
	private long time_boot_ms;

	public void newData(msg_raw_imu msg_imu) {
		xmag = msg_imu.xmag;
		ymag = msg_imu.ymag;
		zmag = msg_imu.zmag;
		xacc = msg_imu.xacc;
		yacc = msg_imu.yacc;
		zacc = msg_imu.zacc;
		xgyro = msg_imu.xgyro;
		ygyro = msg_imu.ygyro;
		zgyro = msg_imu.zgyro;
		time_boot_ms = msg_imu.time_usec;
		drone.notifyDroneEvent(DroneEventsType.MAGNETOMETER);
	}

	public void newData(msg_scaled_imu msg_imu) {
		xmag = msg_imu.xmag;
		ymag = msg_imu.ymag;
		zmag = msg_imu.zmag;
		xacc = msg_imu.xacc;
		yacc = msg_imu.yacc;
		zacc = msg_imu.zacc;
		xgyro = msg_imu.xgyro;
		ygyro = msg_imu.ygyro;
		zgyro = msg_imu.zgyro;
		time_boot_ms = msg_imu.time_boot_ms;
		drone.notifyDroneEvent(DroneEventsType.MAGNETOMETER);
	}

	public void newData(msg_scaled_imu2 msg_imu) {
		xmag = msg_imu.xmag;
		ymag = msg_imu.ymag;
		zmag = msg_imu.zmag;
		xacc = msg_imu.xacc;
		yacc = msg_imu.yacc;
		zacc = msg_imu.zacc;
		xgyro = msg_imu.xgyro;
		ygyro = msg_imu.ygyro;
		xgyro = msg_imu.zgyro;
		time_boot_ms = msg_imu.time_boot_ms;
		drone.notifyDroneEvent(DroneEventsType.MAGNETOMETER);
	}

	public int[] getVector() {
		return new int[] {xmag, ymag, zmag, xacc, yacc, zacc, xgyro, ygyro, zgyro};
	}

	public int getXmag() {
		return xmag;
	}

	public int getYmag() {
		return ymag;
	}

	public int getZmag() {
		return zmag;
	}

	public int getXacc() {
		return xacc;
	}

	public int getYacc() {
		return yacc;
	}

	public int getZacc() {
		return zacc;
	}

	public int getXgyro() {
		return xgyro;
	}

	public int getYgyro() {
		return ygyro;
	}

	public int getZgyro() {
		return zgyro;
	}

	public long getTime_boot_ms() {
		return time_boot_ms;
	}

	public int[] getOffsets() {
		Parameter paramX = drone.getParameters().getParameter("COMPASS_OFS_X");
		Parameter paramY = drone.getParameters().getParameter("COMPASS_OFS_Y");
		Parameter paramZ = drone.getParameters().getParameter("COMPASS_OFS_Z");
		if (paramX == null || paramY == null || paramZ == null) {
			return null;
		}
		return new int[]{paramX.getValue().intValue(), paramY.getValue().intValue(), paramZ.getValue().intValue()};

	}
}
