// MESSAGE TERRAIN_DATA PACKING
package com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega;

import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPacket;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPayload;

/**
* Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST
*/
public class msg_terrain_data extends MAVLinkMessage{

	public static final int MAVLINK_MSG_ID_TERRAIN_DATA = 134;
	public static final int MAVLINK_MSG_LENGTH = 43;
	private static final long serialVersionUID = MAVLINK_MSG_ID_TERRAIN_DATA;
	

 	/**
	* Latitude of SW corner of first grid (degrees *10^7)
	*/
	public int lat; 
 	/**
	* Longitude of SW corner of first grid (in degrees *10^7)
	*/
	public int lon; 
 	/**
	* Grid spacing in meters
	*/
	public short grid_spacing; 
 	/**
	* Terrain data in meters AMSL
	*/
	public short data[] = new short[16]; 
 	/**
	* bit within the terrain request mask
	*/
	public byte gridbit; 

	/**
	 * Generates the payload for a com.dronegcs.mavlink.is.mavlink message for a message of this type
	 * @return
	 */
	public MAVLinkPacket pack(){
		MAVLinkPacket packet = build(MAVLINK_MSG_LENGTH);
		packet.payload.putInt(lat);
		packet.payload.putInt(lon);
		packet.payload.putShort(grid_spacing);
		 for (int i = 0; i < data.length; i++) {
                        packet.payload.putShort(data[i]);
            }
		packet.payload.putByte(gridbit);
		return packet;		
	}

    /**
     * Decode a terrain_data message into this class fields
     *
     * @param payload The message to decode
     */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
	    lat = payload.getInt();
	    lon = payload.getInt();
	    grid_spacing = payload.getShort();
	     for (int i = 0; i < data.length; i++) {
			data[i] = payload.getShort();
		}
	    gridbit = payload.getByte();    
    }

     /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_terrain_data(int sysid){ 		super(sysid);
msgid = MAVLINK_MSG_ID_TERRAIN_DATA;
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a com.dronegcs.mavlink.is.mavlink packet
     * 
     */
    public msg_terrain_data(MAVLinkPacket mavLinkPacket){
        this(mavLinkPacket.sysid);
        unpack(mavLinkPacket.payload);
        //Log.d("MAVLink", "TERRAIN_DATA");
        //Log.d("MAVLINK_MSG_ID_TERRAIN_DATA", toString());
    }
    
          
    /**
     * Returns a string with the MSG name and data
     */
    public String toString(){
    	return "MAVLINK_MSG_ID_TERRAIN_DATA -"+" lat:"+lat+" lon:"+lon+" grid_spacing:"+grid_spacing+" data:"+data+" gridbit:"+gridbit+"";
    }
}
