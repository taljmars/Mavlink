package com.dronegcs.mavlink.is.drone.variables;

import com.dronegcs.mavlink.is.drone.Drone;
import com.dronegcs.mavlink.is.drone.DroneInterfaces;
import com.dronegcs.mavlink.is.drone.DroneInterfaces.DroneEventsType;
import com.dronegcs.mavlink.is.drone.DroneVariable;
import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.msg_autopilot_version;
import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.msg_mission_item;
import com.dronegcs.mavlink.is.protocol.msg_metadata.enums.MAV_CMD;
import com.dronegcs.mavlink.is.protocol.msg_metadata.enums.MAV_FRAME;
import com.dronegcs.mavlink.is.protocol.msgbuilder.MavlinkCapabilities;
import com.dronegcs.mavlink.is.protocol.msgbuilder.MavlinkProtocol;
import com.geo_tools.Coordinate;
import com.geo_tools.GeoTools;
import org.springframework.stereotype.Component;

import javax.annotation.PostConstruct;

@Component
public class Capabilities extends DroneVariable implements DroneInterfaces.OnDroneListener {
	
	private long bytesArray = -1;

    private static int called;
    /**
     * must be called in order to finish object creation
     **/
    public void init() {
        if (called++ > 1)
            throw new RuntimeException("Not a Singleton");
        drone.addDroneListener(this);
    }

	public long getByteArray() {
		return bytesArray;
	}

	public void setCapabilities(msg_autopilot_version msg) {
		this.bytesArray = msg.capabilities;
		drone.notifyDroneEvent(DroneEventsType.AUTOPILOT_VERSION);
	}

	@Override
	public void onDroneEvent(DroneEventsType event, Drone drone) {
		switch (event) {
			case HEARTBEAT_FIRST:
				MavlinkCapabilities.getCapabilities(drone);
				MavlinkProtocol.getSupportedProtocol(drone);
				break;
		}
	}
}
