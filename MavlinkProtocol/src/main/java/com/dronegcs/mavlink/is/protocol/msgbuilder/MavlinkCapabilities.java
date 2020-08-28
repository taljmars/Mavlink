package com.dronegcs.mavlink.is.protocol.msgbuilder;

import com.dronegcs.mavlink.is.drone.Drone;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPacket;
import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.extended.msg_request_autopilot_capabilities;

public class MavlinkCapabilities {

    public static void getCapabilities(Drone drone) {
        msg_request_autopilot_capabilities msg = new msg_request_autopilot_capabilities(drone.getGCS().getId());
        MAVLinkPacket a = msg.pack();
        drone.getMavClient().sendMavPacket(a);
    }
}
