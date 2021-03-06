
package com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega;

import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPacket;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPayload;

/**
* Status of simulation com.dronegcs.gcsis.environment, if used
*/
public class msg_sim_state extends MAVLinkMessage{

	public static final int MAVLINK_MSG_ID_SIM_STATE = 108;
	public static final int MAVLINK_MSG_LENGTH = 84;
	private static final long serialVersionUID = MAVLINK_MSG_ID_SIM_STATE;
	

 	/**
	* True attitude quaternion component 1, w (1 in null-rotation)
	*/
	public float q1; 
 	/**
	* True attitude quaternion component 2, x (0 in null-rotation)
	*/
	public float q2; 
 	/**
	* True attitude quaternion component 3, y (0 in null-rotation)
	*/
	public float q3; 
 	/**
	* True attitude quaternion component 4, z (0 in null-rotation)
	*/
	public float q4; 
 	/**
	* Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
	*/
	public float roll; 
 	/**
	* Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
	*/
	public float pitch; 
 	/**
	* Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
	*/
	public float yaw; 
 	/**
	* X acceleration m/s/s
	*/
	public float xacc; 
 	/**
	* Y acceleration m/s/s
	*/
	public float yacc; 
 	/**
	* Z acceleration m/s/s
	*/
	public float zacc; 
 	/**
	* Angular speed around X axis rad/s
	*/
	public float xgyro; 
 	/**
	* Angular speed around Y axis rad/s
	*/
	public float ygyro; 
 	/**
	* Angular speed around Z axis rad/s
	*/
	public float zgyro; 
 	/**
	* Latitude in degrees
	*/
	public float lat; 
 	/**
	* Longitude in degrees
	*/
	public float lon; 
 	/**
	* Altitude in meters
	*/
	public float alt; 
 	/**
	* Horizontal position standard deviation
	*/
	public float std_dev_horz; 
 	/**
	* Vertical position standard deviation
	*/
	public float std_dev_vert; 
 	/**
	* True velocity in m/s in NORTH direction in earth-fixed NED frame
	*/
	public float vn; 
 	/**
	* True velocity in m/s in EAST direction in earth-fixed NED frame
	*/
	public float ve; 
 	/**
	* True velocity in m/s in DOWN direction in earth-fixed NED frame
	*/
	public float vd; 

	/**
	 * Generates the payload for a com.dronegcs.mavlink.is.mavlink message for a message of this type
	 * @return
	 */
	public MAVLinkPacket pack(){
		MAVLinkPacket packet = build(MAVLINK_MSG_LENGTH);
		packet.payload.putFloat(q1);
		packet.payload.putFloat(q2);
		packet.payload.putFloat(q3);
		packet.payload.putFloat(q4);
		packet.payload.putFloat(roll);
		packet.payload.putFloat(pitch);
		packet.payload.putFloat(yaw);
		packet.payload.putFloat(xacc);
		packet.payload.putFloat(yacc);
		packet.payload.putFloat(zacc);
		packet.payload.putFloat(xgyro);
		packet.payload.putFloat(ygyro);
		packet.payload.putFloat(zgyro);
		packet.payload.putFloat(lat);
		packet.payload.putFloat(lon);
		packet.payload.putFloat(alt);
		packet.payload.putFloat(std_dev_horz);
		packet.payload.putFloat(std_dev_vert);
		packet.payload.putFloat(vn);
		packet.payload.putFloat(ve);
		packet.payload.putFloat(vd);
		return packet;		
	}

    /**
     * Decode a sim_state message into this class fields
     *
     * @param payload The message to decode
     */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
	    q1 = payload.getFloat();
	    q2 = payload.getFloat();
	    q3 = payload.getFloat();
	    q4 = payload.getFloat();
	    roll = payload.getFloat();
	    pitch = payload.getFloat();
	    yaw = payload.getFloat();
	    xacc = payload.getFloat();
	    yacc = payload.getFloat();
	    zacc = payload.getFloat();
	    xgyro = payload.getFloat();
	    ygyro = payload.getFloat();
	    zgyro = payload.getFloat();
	    lat = payload.getFloat();
	    lon = payload.getFloat();
	    alt = payload.getFloat();
	    std_dev_horz = payload.getFloat();
	    std_dev_vert = payload.getFloat();
	    vn = payload.getFloat();
	    ve = payload.getFloat();
	    vd = payload.getFloat();    
    }

     /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_sim_state(int sysid){ 		super(sysid);
    	msgid = MAVLINK_MSG_ID_SIM_STATE;
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a com.dronegcs.mavlink.is.mavlink packet
     * 
     */
    public msg_sim_state(MAVLinkPacket mavLinkPacket){
        this(mavLinkPacket.sysid);
        unpack(mavLinkPacket.payload);
        //Log.d("MAVLink", "SIM_STATE");
        //Log.d("MAVLINK_MSG_ID_SIM_STATE", toString());
    }
    
                                          
    /**
     * Returns a string with the MSG name and data
     */
    public String toString(){
    	return "MAVLINK_MSG_ID_SIM_STATE -"+" q1:"+q1+" q2:"+q2+" q3:"+q3+" q4:"+q4+" roll:"+roll+" pitch:"+pitch+" yaw:"+yaw+" xacc:"+xacc+" yacc:"+yacc+" zacc:"+zacc+" xgyro:"+xgyro+" ygyro:"+ygyro+" zgyro:"+zgyro+" lat:"+lat+" lon:"+lon+" alt:"+alt+" std_dev_horz:"+std_dev_horz+" std_dev_vert:"+std_dev_vert+" vn:"+vn+" ve:"+ve+" vd:"+vd+"";
    }
}
