package com.dronegcs.mavlink.is.drone.profiles;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.generic_tools.logger.Logger;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;
import com.dronegcs.mavlink.is.drone.Drone;
import com.dronegcs.mavlink.is.drone.DroneInterfaces;
import com.dronegcs.mavlink.is.drone.DroneVariable;
import com.dronegcs.mavlink.is.drone.DroneInterfaces.DroneEventsType;
import com.dronegcs.mavlink.is.drone.DroneInterfaces.Handler;
import com.dronegcs.mavlink.is.drone.DroneInterfaces.OnDroneListener;
import com.dronegcs.mavlink.is.drone.parameters.Parameter;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.msg_param_value;
import com.dronegcs.mavlink.is.protocol.msgbuilder.MavLinkParameters;

import javax.validation.constraints.NotNull;

/**
 * Class to manage the communication of parameters to the MAV.
 * 
 * Should be initialized with a MAVLink Object, so the manager can send messages
 * via the MAV link. The function processMessage must be called with every new
 * MAV Message.
 * 
 */
@Component
public class Parameters extends DroneVariable implements OnDroneListener {

	private static final int TIMEOUT = 2000; //TALMA original is 1000;

	private int expectedParams;

	private final HashMap<Integer, Parameter> parameters = new HashMap<Integer, Parameter>();

	private Set<DroneInterfaces.OnParameterManagerListener> parameterListeners;

	@Autowired @NotNull(message = "Internal Error: Failed to get handler")
	public Handler handler;

	@Autowired @NotNull(message = "Internal Error: Failed to get com.dronegcs.gcsis.logger")
	private Logger logger;
	
	public Runnable watchdogCallback = () -> onParameterStreamStopped();

	public final ArrayList<Parameter> parameterList = new ArrayList<Parameter>();
	
	static int called;
	public void init() {
		if (called++ > 1)
			throw new RuntimeException("Not a Singleton");
		parameterListeners = new HashSet<DroneInterfaces.OnParameterManagerListener>();
		drone.addDroneListener(this);
	}

	public void refreshParameters() {
		parameters.clear();
        parameterList.clear();

		if (parameterListeners == null) {
			logger.LogErrorMessege("Error: There are not listeners signed");
			return;
		}
		
		for (DroneInterfaces.OnParameterManagerListener parameterListener : parameterListeners)
			parameterListener.onBeginReceivingParameters();
		
		MavLinkParameters.requestParametersList(drone);
		resetWatchdog();
	}

    public List<Parameter> getParametersList(){
        return parameterList;
    }
    
    public int getLoadedDownloadedParameters() {
    	return parameters.size();
    }

	/**
	 * Try to process a Mavlink message if it is a parameter related message
	 * 
	 * @param msg
	 *            Mavlink message to process
	 * @return Returns true if the message has been processed
	 */
	public boolean processMessage(MAVLinkMessage msg) {
		if (msg.msgid == msg_param_value.MAVLINK_MSG_ID_PARAM_VALUE) {
			processReceivedParam((msg_param_value) msg);
			return true;
		}
		return false;
	}

	private void processReceivedParam(msg_param_value m_value) {
		// collect params in parameter list
		Parameter param = new Parameter(m_value);
		parameters.put((int) m_value.param_index, param);

		expectedParams = m_value.param_count;

		// update listener
		if (parameterListeners != null)
			for (DroneInterfaces.OnParameterManagerListener parameterListener : parameterListeners)
				parameterListener.onParameterReceived(param, m_value.param_index, m_value.param_count);
		
		// Are all parameters here? Notify the listener with the parameters
		if (parameters.size() >= m_value.param_count) {
            parameterList.clear();
			for (int key : parameters.keySet()) {
				parameterList.add(parameters.get(key));
			}
			killWatchdog();
			logger.LogGeneralMessege("Parameters finished!");

			if (parameterListeners != null) {
				for (DroneInterfaces.OnParameterManagerListener parameterListener : parameterListeners)
					parameterListener.onEndReceivingParameters(parameterList);
			}
		} else {
			resetWatchdog();
		}
		drone.notifyDroneEvent(DroneEventsType.PARAMETER);
	}

	private void reRequestMissingParams(int howManyParams) {
		for (int i = 0; i < howManyParams; i++) {
			if (!parameters.containsKey(i)) {
				MavLinkParameters.readParameter(drone, i);
			}
		}
	}

	public void sendParameter(Parameter parameter) {
		MavLinkParameters.sendParameter(drone, parameter);
	}

	public void ReadParameter(String name) {
		MavLinkParameters.readParameter(drone, name);
	}

	public Parameter getParameter(String name) {
		for (int key : parameters.keySet()) {
			if (parameters.get(key).name.equalsIgnoreCase(name))
				return parameters.get(key);
		}
		return null;
	}

	public Parameter getLastParameter() {
		if (parameters.size() > 0)
			return parameters.get(parameters.size() - 1);

		return null;
	}

	private void onParameterStreamStopped() {
		reRequestMissingParams(expectedParams);
		resetWatchdog();
	}

	private void resetWatchdog() {
		handler.removeCallbacks(watchdogCallback);
		handler.postDelayed(watchdogCallback, TIMEOUT);
	}

	private void killWatchdog() {
		handler.removeCallbacks(watchdogCallback);
	}	
	
	@Override
	public void onDroneEvent(DroneEventsType event, Drone drone) {
		switch (event) {
		case HEARTBEAT_FIRST:
			if (!drone.getState().isFlying()) {
				System.out.println(getClass().getName() + " First HB Packet");
				//refreshParameters();
			}
			break;
		case DISCONNECTED:
		case HEARTBEAT_TIMEOUT:
			killWatchdog();
			break;
		default:
			break;

		}
	}
	
	public int getExpectedParameterAmount() {
		return expectedParams;
	}

	public void addParameterListener(DroneInterfaces.OnParameterManagerListener parameterListener) {
		System.out.println(getClass().getName() + " Setting new paramter listener " + parameterListener.getClass());
		this.parameterListeners.add(parameterListener);
	}

	public int getPrecentageComplete() {
		return (int) (((double) (getLoadedDownloadedParameters()) / getExpectedParameterAmount()) * 100);
	}
}