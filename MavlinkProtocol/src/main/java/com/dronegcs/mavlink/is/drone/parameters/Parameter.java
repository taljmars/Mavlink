package com.dronegcs.mavlink.is.drone.parameters;

import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.msg_param_value;

import java.io.Serializable;
import java.text.DecimalFormat;

public class Parameter implements Comparable<Parameter>, Serializable {

	/**
	 * 
	 */
	private static final long serialVersionUID = -4968235225796045915L;
	
	public String name;
	public double value;
	public int type;
	public String description;

	private final static DecimalFormat format = (DecimalFormat) DecimalFormat.getInstance();
	static {
		format.applyPattern("0.###");
	}

	public Parameter(String name, double value, int type, String description) {
		this.name = name;
		this.value = value;
		this.type = type;
		this.description = description;
	}

	public Parameter(msg_param_value m_value, String description) {
		this(m_value.getParam_Id(), m_value.param_value, m_value.param_type, description);
	}

//	public Parameter(String name, Double value) {
//		this(name, value, 0); // TODO Setting type to Zero may cause an error
//	}
//
//	public Parameter(String name) {
//		this(name, 0, 0); // TODO Setting type to Zero may cause an error
//	}

	public String getValue() {
		return format.format(value);
	}

	public static void checkParameterName(String name) throws Exception {
		if (name.equals("SYSID_SW_MREV")) {
			throw new Exception("ExludedName");
		} else if (name.contains("WP_TOTAL")) {
			throw new Exception("ExludedName");
		} else if (name.contains("CMD_TOTAL")) {
			throw new Exception("ExludedName");
		} else if (name.contains("FENCE_TOTAL")) {
			throw new Exception("ExludedName");
		} else if (name.contains("SYS_NUM_RESETS")) {
			throw new Exception("ExludedName");
		} else if (name.contains("ARSPD_OFFSET")) {
			throw new Exception("ExludedName");
		} else if (name.contains("GND_ABS_PRESS")) {
			throw new Exception("ExludedName");
		} else if (name.contains("GND_TEMP")) {
			throw new Exception("ExludedName");
		} else if (name.contains("CMD_INDEX")) {
			throw new Exception("ExludedName");
		} else if (name.contains("LOG_LASTFILE")) {
			throw new Exception("ExludedName");
		} else if (name.contains("FORMAT_VERSION")) {
			throw new Exception("ExludedName");
		} else {
		}
	}

	public String getDescription() {
		return description;
	}

	public void setDescription(String description) {
		this.description = description;
	}

	public static DecimalFormat getFormat() {
		return format;
	}

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Parameter)) return false;

        Parameter parameter = (Parameter) o;

        return (name == null ? parameter.name == null : name.equals(parameter.name));
    }

    @Override
    public int hashCode() {
        return name != null ? name.hashCode() : 0;
    }

    @Override
    public int compareTo(Parameter another) {
        return name.compareTo(another.name);
    }

	@Override
	public String toString() {
		return "Parameter{" +
				"name='" + name + '\'' +
				", value=" + value +
				", type=" + type +
				'}';
	}
}
