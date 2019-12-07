package com.dronegcs.mavlink.is.protocol.msg_metadata;

import com.dronegcs.mavlink.is.protocol.msg_metadata.enums.MAV_FRAME_TYPE;
import com.dronegcs.mavlink.is.protocol.msg_metadata.enums.MAV_TYPE;

import java.util.ArrayList;
import java.util.List;

public enum ApmFrameTypes {
	PLUS ("Plus", MAV_FRAME_TYPE.MOTOR_FRAME_TYPE_PLUS),
	X ("X", MAV_FRAME_TYPE.MOTOR_FRAME_TYPE_X),
	V ("V", MAV_FRAME_TYPE.MOTOR_FRAME_TYPE_V),
	H ("H", MAV_FRAME_TYPE.MOTOR_FRAME_TYPE_H),
	VTAIL ("V-Tail", MAV_FRAME_TYPE.MOTOR_FRAME_TYPE_VTAIL),
	ATAIL ("A-Tail", MAV_FRAME_TYPE.MOTOR_FRAME_TYPE_ATAIL),
	Y6B ("Y6B", MAV_FRAME_TYPE.MOTOR_FRAME_TYPE_Y6B);

    private final String name;
	private final int type;

	ApmFrameTypes(String name, int type){
		this.name = name;
		this.type = type;
	}

	public String getName() {
		return name;
	}

	public int getType() {
		return type;
	}

	public static ApmFrameTypes getFrameType(int typeId) {
		for (ApmFrameTypes type : ApmFrameTypes.values()) {
			if (typeId == type.getType())
				return type;
		}

		return null;
	}

	public static List<ApmFrameTypes> getFrameTypesList() {
		List<ApmFrameTypes> list = new ArrayList<ApmFrameTypes>();

		for (ApmFrameTypes type : ApmFrameTypes.values()) {
			list.add(type);
		}
		return list;
	}

	@Override
	public String toString() {
		return name;
	}

}
