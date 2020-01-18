// MESSAGE HWSTATUS PACKING
package com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega;

import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPacket;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPayload;

/**
* The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
*/
public class msg_local_position_ned_cov extends MAVLinkMessage{

	public static final int MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV = 64;
	public static final int MAVLINK_MSG_LENGTH = 181;
	private static final long serialVersionUID = MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV;
	

 	/**
	* Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown. Commonly filled by the precision time source of a GPS receiver.
	*/
	public long time_utc; 
 	/**
	* Timestamp (milliseconds since system boot)
	*/
	public int time_boot_ms; 
 	/**
	* X Position
	*/
	public float x; 
 	/**
	* Y Position
	*/
	public float y; 
 	/**
	* Z Position
	*/
	public float z; 
 	/**
	* X Speed
	*/
	public float vx; 
 	/**
	* Y Speed
	*/
	public float vy; 
 	/**
	* Z Speed
	*/
	public float vz; 
 	/**
	* Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
	*/
	public float covariance[] = new float[36]; 
 	/**
	* Class id of the estimator this estimate originated from.
	*/
	public byte estimator_type; 

	/**
	 * Generates the payload for a com.dronegcs.mavlink.is.mavlink message for a message of this type
	 * @return
	 */
	public MAVLinkPacket pack(){
		MAVLinkPacket packet = build(MAVLINK_MSG_LENGTH);
		packet.payload.putLong(time_utc);
		packet.payload.putInt(time_boot_ms);
		packet.payload.putFloat(x);
		packet.payload.putFloat(y);
		packet.payload.putFloat(z);
		packet.payload.putFloat(vx);
		packet.payload.putFloat(vy);
		packet.payload.putFloat(vz);
		 for (int i = 0; i < covariance.length; i++) {
                        packet.payload.putFloat(covariance[i]);
            }
		packet.payload.putByte(estimator_type);
		return packet;		
	}

    /**
     * Decode a local_position_ned_cov message into this class fields
     *
     * @param payload The message to decode
     */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
	    time_utc = payload.getLong();
	    time_boot_ms = payload.getInt();
	    x = payload.getFloat();
	    y = payload.getFloat();
	    z = payload.getFloat();
	    vx = payload.getFloat();
	    vy = payload.getFloat();
	    vz = payload.getFloat();
	     for (int i = 0; i < covariance.length; i++) {
			covariance[i] = payload.getFloat();
		}
	    estimator_type = payload.getByte();    
    }

     /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_local_position_ned_cov(int sysid){
		super(sysid);
		msgid = MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV;
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a com.dronegcs.mavlink.is.mavlink packet
     * 
     */
    public msg_local_position_ned_cov(MAVLinkPacket mavLinkPacket){
        this(mavLinkPacket.sysid);
        unpack(mavLinkPacket.payload);
        //Log.d("MAVLink", "LOCAL_POSITION_NED_COV");
        //Log.d("MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV", toString());
    }
    
                    
    /**
     * Returns a string with the MSG name and data
     */
    public String toString(){
    	return "MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV -"+" time_utc:"+time_utc+" time_boot_ms:"+time_boot_ms+" x:"+x+" y:"+y+" z:"+z+" vx:"+vx+" vy:"+vy+" vz:"+vz+" covariance:"+covariance+" estimator_type:"+estimator_type+"";
    }
}
