// MESSAGE FILE_TRANSFER_RES PACKING
package com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega;

import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPacket;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPayload;

/**
* File transfer result
*/
public class msg_file_transfer_res extends MAVLinkMessage{

	public static final int MAVLINK_MSG_ID_FILE_TRANSFER_RES = 112;
	public static final int MAVLINK_MSG_LENGTH = 9;
	private static final long serialVersionUID = MAVLINK_MSG_ID_FILE_TRANSFER_RES;
	

 	/**
	* Unique transfer ID
	*/
	public long transfer_uid; 
 	/**
	* 0: OK, 1: not permitted, 2: bad path / file name, 3: no space left on device
	*/
	public byte result; 

	/**
	 * Generates the payload for a com.dronegcs.mavlink.is.mavlink message for a message of this type
	 * @return
	 */
	public MAVLinkPacket pack(){
		MAVLinkPacket packet = build(MAVLINK_MSG_LENGTH);
		packet.payload.putLong(transfer_uid);
		packet.payload.putByte(result);
		return packet;		
	}

    /**
     * Decode a file_transfer_res message into this class fields
     *
     * @param payload The message to decode
     */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
	    transfer_uid = payload.getLong();
	    result = payload.getByte();    
    }

     /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_file_transfer_res(int sysid){
		super(sysid);
    	msgid = MAVLINK_MSG_ID_FILE_TRANSFER_RES;
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a com.dronegcs.mavlink.is.mavlink packet
     * 
     */
    public msg_file_transfer_res(MAVLinkPacket mavLinkPacket){
        this(mavLinkPacket.sysid);
        unpack(mavLinkPacket.payload);
        //Log.d("MAVLink", "FILE_TRANSFER_RES");
        //Log.d("MAVLINK_MSG_ID_FILE_TRANSFER_RES", toString());
    }
    
    
    /**
     * Returns a string with the MSG name and data
     */
    public String toString(){
    	return "MAVLINK_MSG_ID_FILE_TRANSFER_RES -"+" transfer_uid:"+transfer_uid+" result:"+result+"";
    }
}
