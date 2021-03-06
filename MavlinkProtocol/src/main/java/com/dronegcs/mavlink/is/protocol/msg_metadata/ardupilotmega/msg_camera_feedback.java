// MESSAGE CAMERA_FEEDBACK PACKING
package com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega;

import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPacket;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPayload;
/**
* Camera Capture Feedback
*/
public class msg_camera_feedback extends MAVLinkMessage{

	public static final int MAVLINK_MSG_ID_CAMERA_FEEDBACK = 180;
	public static final int MAVLINK_MSG_LENGTH = 45;
	private static final long serialVersionUID = MAVLINK_MSG_ID_CAMERA_FEEDBACK;
	

 	/**
	* Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB)
	*/
	public long time_usec; 
 	/**
	* Latitude in (deg * 1E7)
	*/
	public int lat; 
 	/**
	* Longitude in (deg * 1E7)
	*/
	public int lng; 
 	/**
	* Altitude Absolute (meters AMSL)
	*/
	public float alt_msl; 
 	/**
	* Altitude Relative (meters above HOME location)
	*/
	public float alt_rel; 
 	/**
	* Camera Roll angle (earth frame, degrees, +-180)
	*/
	public float roll; 
 	/**
	* Camera Pitch angle (earth frame, degrees, +-180)
	*/
	public float pitch; 
 	/**
	* Camera Yaw (earth frame, degrees, 0-360, true)
	*/
	public float yaw; 
 	/**
	* Focal Length (mm)
	*/
	public float foc_len; 
 	/**
	* Image index
	*/
	public short img_idx; 
 	/**
	* System ID
	*/
	public byte target_system; 
 	/**
	* Camera ID
	*/
	public byte cam_idx; 
 	/**
	* See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask
	*/
	public byte flags; 

	/**
	 * Generates the payload for a com.dronegcs.mavlink.is.mavlink message for a message of this type
	 * @return
	 */
	public MAVLinkPacket pack(){
		MAVLinkPacket packet = build(MAVLINK_MSG_LENGTH);
		packet.payload.putLong(time_usec);
		packet.payload.putInt(lat);
		packet.payload.putInt(lng);
		packet.payload.putFloat(alt_msl);
		packet.payload.putFloat(alt_rel);
		packet.payload.putFloat(roll);
		packet.payload.putFloat(pitch);
		packet.payload.putFloat(yaw);
		packet.payload.putFloat(foc_len);
		packet.payload.putShort(img_idx);
		packet.payload.putByte(target_system);
		packet.payload.putByte(cam_idx);
		packet.payload.putByte(flags);
		return packet;		
	}

    /**
     * Decode a camera_feedback message into this class fields
     *
     * @param payload The message to decode
     */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
	    time_usec = payload.getLong();
	    lat = payload.getInt();
	    lng = payload.getInt();
	    alt_msl = payload.getFloat();
	    alt_rel = payload.getFloat();
	    roll = payload.getFloat();
	    pitch = payload.getFloat();
	    yaw = payload.getFloat();
	    foc_len = payload.getFloat();
	    img_idx = payload.getShort();
	    target_system = payload.getByte();
	    cam_idx = payload.getByte();
	    flags = payload.getByte();    
    }

     /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_camera_feedback(int sysid){
		super(sysid);
		msgid = MAVLINK_MSG_ID_CAMERA_FEEDBACK;
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a com.dronegcs.mavlink.is.mavlink packet
     * 
     */
    public msg_camera_feedback(MAVLinkPacket mavLinkPacket){
        this(mavLinkPacket.sysid);
        unpack(mavLinkPacket.payload);
        //Log.d("MAVLink", "CAMERA_FEEDBACK");
        //Log.d("MAVLINK_MSG_ID_CAMERA_FEEDBACK", toString());
    }
    
                          
    /**
     * Returns a string with the MSG name and data
     */
    public String toString(){
    	return "MAVLINK_MSG_ID_CAMERA_FEEDBACK -"+" time_usec:"+time_usec+" lat:"+lat+" lng:"+lng+" alt_msl:"+alt_msl+" alt_rel:"+alt_rel+" roll:"+roll+" pitch:"+pitch+" yaw:"+yaw+" foc_len:"+foc_len+" img_idx:"+img_idx+" target_system:"+target_system+" cam_idx:"+cam_idx+" flags:"+flags+"";
    }
}
