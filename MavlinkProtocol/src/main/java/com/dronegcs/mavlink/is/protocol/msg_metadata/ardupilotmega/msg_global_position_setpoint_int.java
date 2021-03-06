// MESSAGE GLOBAL_POSITION_SETPOINT_INT PACKING
package com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega;

import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPacket;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPayload;
/**
* Transmit the current local setpoint of the controller to other MAVs (collision avoidance) and to the GCS.
*/
public class msg_global_position_setpoint_int extends MAVLinkMessage{

	public static final int MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT = 52;
	public static final int MAVLINK_MSG_LENGTH = 15;
	private static final long serialVersionUID = MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT;
	

 	/**
	* Latitude (WGS84), in degrees * 1E7
	*/
	public int latitude; 
 	/**
	* Longitude (WGS84), in degrees * 1E7
	*/
	public int longitude; 
 	/**
	* Altitude (WGS84), in meters * 1000 (positive for up)
	*/
	public int altitude; 
 	/**
	* Desired yaw angle in degrees * 100
	*/
	public short yaw; 
 	/**
	* Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
	*/
	public byte coordinate_frame; 

	/**
	 * Generates the payload for a com.dronegcs.mavlink.is.mavlink message for a message of this type
	 * @return
	 */
	public MAVLinkPacket pack(){
		MAVLinkPacket packet = build(MAVLINK_MSG_LENGTH);
		packet.payload.putInt(latitude);
		packet.payload.putInt(longitude);
		packet.payload.putInt(altitude);
		packet.payload.putShort(yaw);
		packet.payload.putByte(coordinate_frame);
		return packet;		
	}

    /**
     * Decode a global_position_setpoint_int message into this class fields
     *
     * @param payload The message to decode
     */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
	    latitude = payload.getInt();
	    longitude = payload.getInt();
	    altitude = payload.getInt();
	    yaw = payload.getShort();
	    coordinate_frame = payload.getByte();    
    }

     /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_global_position_setpoint_int(int sysid){ 		super(sysid);
    	msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT;
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a com.dronegcs.mavlink.is.mavlink packet
     * 
     * @param mavLinkPacket
     */
    public msg_global_position_setpoint_int(MAVLinkPacket mavLinkPacket){
        this(mavLinkPacket.sysid);
        unpack(mavLinkPacket.payload);
        //Log.d("MAVLink", "GLOBAL_POSITION_SETPOINT_INT");
        //Log.d("MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT", toString());
    }
    
          
    /**
     * Returns a string with the MSG name and data
     */
    public String toString(){
    	return "MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT -"+" latitude:"+latitude+" longitude:"+longitude+" altitude:"+altitude+" yaw:"+yaw+" coordinate_frame:"+coordinate_frame+"";
    }
}
