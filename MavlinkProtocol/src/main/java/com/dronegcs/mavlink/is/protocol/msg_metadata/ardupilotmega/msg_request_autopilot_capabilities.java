
package com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega;

import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPacket;

/**
* The system time is the time of the master clock, typically the computer clock of the main onboard computer.
*/
public class msg_request_autopilot_capabilities extends msg_command_long {

	public static final int MAVLINK_MSG_REQUEST_AUTOPILOT_CAPABILITIES = 520;
	public static final int MAVLINK_MSG_LENGTH = 1;
	private static final long serialVersionUID = MAVLINK_MSG_REQUEST_AUTOPILOT_CAPABILITIES;

     /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_request_autopilot_capabilities(){
    	super();
    	command = MAVLINK_MSG_REQUEST_AUTOPILOT_CAPABILITIES;
    	param1 = 1;
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a com.dronegcs.mavlink.is.mavlink packet
     *
     */
    public msg_request_autopilot_capabilities(MAVLinkPacket mavLinkPacket){
        this.sysid = mavLinkPacket.sysid;
        this.compid = mavLinkPacket.compid;
        this.command = MAVLINK_MSG_REQUEST_AUTOPILOT_CAPABILITIES;
        this.param1 = 1;
        //Log.d("MAVLink", "SYSTEM_TIME");
        //Log.d("MAVLINK_MSG_ID_SYSTEM_TIME", toString());
    }
    
    
    /**
     * Returns a string with the MSG name and data
     */
    public String toString(){
    	return "MAVLINK_MSG_REQUEST_AUTOPILOT_CAPABILITIES";
    }
}
