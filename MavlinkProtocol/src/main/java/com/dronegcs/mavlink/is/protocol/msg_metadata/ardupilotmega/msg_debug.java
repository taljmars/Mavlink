// MESSAGE DEBUG PACKING
package com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega;

import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPacket;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPayload;

/**
* Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
*/
public class msg_debug extends MAVLinkMessage{

	public static final int MAVLINK_MSG_ID_DEBUG = 254;
	public static final int MAVLINK_MSG_LENGTH = 9;
	private static final long serialVersionUID = MAVLINK_MSG_ID_DEBUG;
	

 	/**
	* Timestamp (milliseconds since system boot)
	*/
	public int time_boot_ms; 
 	/**
	* DEBUG value
	*/
	public float value; 
 	/**
	* index of debug variable
	*/
	public byte ind; 

	/**
	 * Generates the payload for a com.dronegcs.mavlink.is.mavlink message for a message of this type
	 * @return
	 */
	public MAVLinkPacket pack(){
		MAVLinkPacket packet = build(MAVLINK_MSG_LENGTH);
		packet.payload.putInt(time_boot_ms);
		packet.payload.putFloat(value);
		packet.payload.putByte(ind);
		return packet;		
	}

    /**
     * Decode a debug message into this class fields
     *
     * @param payload The message to decode
     */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
	    time_boot_ms = payload.getInt();
	    value = payload.getFloat();
	    ind = payload.getByte();    
    }

     /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_debug(int sysid){ 		super(sysid);
    	msgid = MAVLINK_MSG_ID_DEBUG;
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a com.dronegcs.mavlink.is.mavlink packet
     * 
     */
    public msg_debug(MAVLinkPacket mavLinkPacket){
        this(mavLinkPacket.sysid);
        unpack(mavLinkPacket.payload);
        //Log.d("MAVLink", "DEBUG");
        //Log.d("MAVLINK_MSG_ID_DEBUG", toString());
    }
    
      
    /**
     * Returns a string with the MSG name and data
     */
    public String toString(){
    	return "MAVLINK_MSG_ID_DEBUG -"+" time_boot_ms:"+time_boot_ms+" value:"+value+" ind:"+ind+"";
    }
}
