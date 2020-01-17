// MESSAGE COMMAND_INT PACKING
package com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega;

import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPacket;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPayload;
/**
* Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value.
*/
public class msg_command_int extends MAVLinkMessage{

	public static final int MAVLINK_MSG_ID_COMMAND_INT = 75;
	public static final int MAVLINK_MSG_LENGTH = 35;
	private static final long serialVersionUID = MAVLINK_MSG_ID_COMMAND_INT;
	

 	/**
	* PARAM1, see MAV_CMD enum
	*/
	public float param1; 
 	/**
	* PARAM2, see MAV_CMD enum
	*/
	public float param2; 
 	/**
	* PARAM3, see MAV_CMD enum
	*/
	public float param3; 
 	/**
	* PARAM4, see MAV_CMD enum
	*/
	public float param4; 
 	/**
	* PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	*/
	public int x; 
 	/**
	* PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
	*/
	public int y; 
 	/**
	* PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
	*/
	public float z; 
 	/**
	* The scheduled action for the droneMission item. see MAV_CMD in common.xml MAVLink specs
	*/
	public short command; 
 	/**
	* System ID
	*/
	public byte target_system; 
 	/**
	* Component ID
	*/
	public byte target_component; 
 	/**
	* The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
	*/
	public byte frame; 
 	/**
	* false:0, true:1
	*/
	public byte current; 
 	/**
	* autocontinue to next wp
	*/
	public byte autocontinue; 

	/**
	 * Generates the payload for a com.dronegcs.mavlink.is.mavlink message for a message of this type
	 * @return
	 */
	public MAVLinkPacket pack(){
		MAVLinkPacket packet = new MAVLinkPacket();
		packet.len = MAVLINK_MSG_LENGTH;
		packet.sysid = 255;
		packet.compid = 190;
		packet.msgid = MAVLINK_MSG_ID_COMMAND_INT;
		packet.payload.putFloat(param1);
		packet.payload.putFloat(param2);
		packet.payload.putFloat(param3);
		packet.payload.putFloat(param4);
		packet.payload.putInt(x);
		packet.payload.putInt(y);
		packet.payload.putFloat(z);
		packet.payload.putShort(command);
		packet.payload.putByte(target_system);
		packet.payload.putByte(target_component);
		packet.payload.putByte(frame);
		packet.payload.putByte(current);
		packet.payload.putByte(autocontinue);
		return packet;		
	}

    /**
     * Decode a command_int message into this class fields
     *
     * @param payload The message to decode
     */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
	    param1 = payload.getFloat();
	    param2 = payload.getFloat();
	    param3 = payload.getFloat();
	    param4 = payload.getFloat();
	    x = payload.getInt();
	    y = payload.getInt();
	    z = payload.getFloat();
	    command = payload.getShort();
	    target_system = payload.getByte();
	    target_component = payload.getByte();
	    frame = payload.getByte();
	    current = payload.getByte();
	    autocontinue = payload.getByte();    
    }

     /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_command_int(int sysid){
		super(sysid);
		msgid = MAVLINK_MSG_ID_COMMAND_INT;
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a com.dronegcs.mavlink.is.mavlink packet
     * 
     */
    public msg_command_int(MAVLinkPacket mavLinkPacket){
        this(mavLinkPacket.sysid);
        unpack(mavLinkPacket.payload);
        //Log.d("MAVLink", "COMMAND_INT");
        //Log.d("MAVLINK_MSG_ID_COMMAND_INT", toString());
    }
    
                          
    /**
     * Returns a string with the MSG name and data
     */
    public String toString(){
    	return "MAVLINK_MSG_ID_COMMAND_INT -"+" param1:"+param1+" param2:"+param2+" param3:"+param3+" param4:"+param4+" x:"+x+" y:"+y+" z:"+z+" command:"+command+" target_system:"+target_system+" target_component:"+target_component+" frame:"+frame+" current:"+current+" autocontinue:"+autocontinue+"";
    }
}
