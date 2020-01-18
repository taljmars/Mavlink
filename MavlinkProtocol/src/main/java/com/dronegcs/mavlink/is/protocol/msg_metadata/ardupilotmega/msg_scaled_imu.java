// MESSAGE SCALED_IMU PACKING
package com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega;

import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPacket;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPayload;

/**
* The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
*/
public class msg_scaled_imu extends MAVLinkMessage{

	public static final int MAVLINK_MSG_ID_SCALED_IMU = 26;
	public static final int MAVLINK_MSG_LENGTH = 22;
	private static final long serialVersionUID = MAVLINK_MSG_ID_SCALED_IMU;
	

 	/**
	* Timestamp (milliseconds since system boot)
	*/
	public int time_boot_ms; 
 	/**
	* X acceleration (mg)
	*/
	public short xacc; 
 	/**
	* Y acceleration (mg)
	*/
	public short yacc; 
 	/**
	* Z acceleration (mg)
	*/
	public short zacc; 
 	/**
	* Angular speed around X axis (millirad /sec)
	*/
	public short xgyro; 
 	/**
	* Angular speed around Y axis (millirad /sec)
	*/
	public short ygyro; 
 	/**
	* Angular speed around Z axis (millirad /sec)
	*/
	public short zgyro; 
 	/**
	* X Magnetic field (milli tesla)
	*/
	public short xmag; 
 	/**
	* Y Magnetic field (milli tesla)
	*/
	public short ymag; 
 	/**
	* Z Magnetic field (milli tesla)
	*/
	public short zmag; 

	/**
	 * Generates the payload for a com.dronegcs.mavlink.is.mavlink message for a message of this type
	 * @return
	 */
	public MAVLinkPacket pack(){
		MAVLinkPacket packet = build(MAVLINK_MSG_LENGTH);
		packet.payload.putInt(time_boot_ms);
		packet.payload.putShort(xacc);
		packet.payload.putShort(yacc);
		packet.payload.putShort(zacc);
		packet.payload.putShort(xgyro);
		packet.payload.putShort(ygyro);
		packet.payload.putShort(zgyro);
		packet.payload.putShort(xmag);
		packet.payload.putShort(ymag);
		packet.payload.putShort(zmag);
		return packet;		
	}

    /**
     * Decode a scaled_imu message into this class fields
     *
     * @param payload The message to decode
     */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
	    time_boot_ms = payload.getInt();
	    xacc = payload.getShort();
	    yacc = payload.getShort();
	    zacc = payload.getShort();
	    xgyro = payload.getShort();
	    ygyro = payload.getShort();
	    zgyro = payload.getShort();
	    xmag = payload.getShort();
	    ymag = payload.getShort();
	    zmag = payload.getShort();    
    }

     /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_scaled_imu(int sysid){
		super(sysid);
    	msgid = MAVLINK_MSG_ID_SCALED_IMU;
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a com.dronegcs.mavlink.is.mavlink packet
     * 
     */
    public msg_scaled_imu(MAVLinkPacket mavLinkPacket){
        this(mavLinkPacket.sysid);
        unpack(mavLinkPacket.payload);
        //Log.d("MAVLink", "SCALED_IMU");
        //Log.d("MAVLINK_MSG_ID_SCALED_IMU", toString());
    }
    
                    
    /**
     * Returns a string with the MSG name and data
     */
    public String toString(){
    	return "MAVLINK_MSG_ID_SCALED_IMU -"+" time_boot_ms:"+time_boot_ms+" xacc:"+xacc+" yacc:"+yacc+" zacc:"+zacc+" xgyro:"+xgyro+" ygyro:"+ygyro+" zgyro:"+zgyro+" xmag:"+xmag+" ymag:"+ymag+" zmag:"+zmag+"";
    }
}
