package com.dronegcs.tester;

import com.dronegcs.mavlink.core.gcs.GCSHeartbeat;
import com.dronegcs.mavlink.is.drone.Drone;
import com.dronegcs.mavlink.is.drone.DroneInterfaces;
import com.dronegcs.mavlink.is.drone.parameters.Parameter;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkMessage;
import com.dronegcs.mavlink.is.protocol.msg_metadata.MAVLinkPacket;
import com.dronegcs.mavlink.is.protocol.msgbuilder.WaypointManager;
import com.generic_tools.devices.SerialConnection;
import com.generic_tools.environment.Environment;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;

import javax.annotation.PostConstruct;
import java.io.IOException;
import java.net.URISyntaxException;
import java.util.List;
import java.util.Scanner;

/**
 * Created by taljmars on 4/5/17.
 */
@Component
public class MavlinkTester implements DroneInterfaces.OnParameterManagerListener, DroneInterfaces.OnDroneListener, DroneInterfaces.OnWaypointManagerListener {

    @Autowired
    private GCSHeartbeat gcsHeartbeat;

    @Autowired
    private Drone drone;

    @Autowired
    private SerialConnection serialConnection;

    @Autowired
    private Environment environment;

    @PostConstruct
    private void init() throws URISyntaxException {
        drone.addDroneListener(this);
        drone.getParameters().addParameterListener(this);
        drone.getWaypointManager().addWaypointManagerListener(this);
        environment.setBaseRunningDirectoryByClass(".");
    }

    public void go() throws IOException {
        System.out.println("Start Mavlink Drone Tester");
        byte[] buff = new byte[100];
        Scanner reader = new Scanner(System.in);
        while (reader.hasNext()) {
            String s = reader.next();
            System.out.println("Received '" + s + "' from user");
            switch (s) {
                case "listports":
                    listPort();
                    break;
                case "connect":
                    connect();
                    break;
                case "disconnect":
                    disconnect();
                    break;
                case "sync":
                    sync();
                    break;
                case "getver":
                    checkVerson();
                    break;
                case "push":
                    pushWaypoints();
                    break;
                case "fetch":
                    fetchWaypoints();
                    break;
                case "ping":
                    ping();
                    break;
                case "exit":
                    System.exit(0);
            }
        }
    }

    private void checkVerson() {
        //MAV_CMD_REQUEST_PROTOCOL_VERSION

        MAVLinkPacket mavLinkPacket = new MAVLinkPacket();
        mavLinkPacket.msgid = 410;
        drone.getMavClient().sendMavPacket(mavLinkPacket);
    }

    private void listPort() {
        Object[] ports = serialConnection.listPorts();
        if (ports.length == 0) {
            System.out.println("No ports found");
            return;
        }

        System.out.println("Available ports:");
        for (int i = 0 ; i < ports.length ; i++) {
            System.out.println("-> " + ports[i]);
        }

        System.out.println("Supported Bauds: " + serialConnection.baudList());
        System.out.println("Default Baud: " + serialConnection.getDefaultBaud());
    }

    private void connect() {
        Object[] ports = serialConnection.listPorts();
        if (ports.length == 0) {
            System.out.println("No ports found");
            return;
        }
        serialConnection.setPortName((String) ports[1]);
        //serialConnection.setBaud(56700);
        serialConnection.setBaud(115200);

        drone.getMavClient().connect();
    }

    private void disconnect() {
        drone.getMavClient().disconnect();
    }

    private void sync() {
        System.out.println("Start HB");
        gcsHeartbeat.setActive(true);
        System.out.println("Sync Parameters");
        drone.getParameters().refreshParameters();
    }

    private void fetchWaypoints() {
        System.out.println("Fetch Waypoints");
        drone.getWaypointManager().getWaypoints();
    }

    private void pushWaypoints() {
        System.out.println("Push Waypoints");
        if (drone.getDroneMission().makeAndUploadDronie() == -1) {
            System.out.println("Failed to build and upload mission");
        }
    }

    private void ping() {
        System.out.println("Ping Drone");
        if (drone.isConnectionAlive()) {
            drone.getMavClient().ping();
        }
    }

    private void sleep(int seconds) {
        try {
            Thread.sleep(1000 * seconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) throws IOException {
        if (args.length != 2) {
            System.err.println("Missing args: usage: <logdir> <confdir>");
            System.exit(-1);
        }
        System.setProperty("LOGS.DIR", args[0]);
        System.setProperty("CONF.DIR", args[1]);
        System.out.println("Logs directory set to: " + System.getProperty("LOGS.DIR"));
        System.out.println("Configuration directory set to: " + System.getProperty("CONF.DIR"));
        MavlinkTester mavlinkTester = AppConfig.context.getBean(MavlinkTester.class);
        mavlinkTester.go();
    }

    @Override
    public void onBeginReceivingParameters() {
        System.out.println("Start receiving parameters");
    }

    @Override
    public void onParameterReceived(Parameter parameter, int index, int count) {
        System.out.println("Received " + index + "/" + count + " param=" + parameter.name + " description=" + parameter.getDescription());
    }

    @Override
    public void onEndReceivingParameters(List<Parameter> parameter) {
        System.out.println("Finish receiving parameters, amount=" + parameter.size());
    }

    @Override
    public void onDroneEvent(DroneInterfaces.DroneEventsType event, Drone drone) {
        System.out.println(System.currentTimeMillis() + " " + event);
    }

    @Override
    public void onBeginWaypointEvent(WaypointManager.WaypointEvent_Type wpEvent) {
        System.out.println("Start getting waypoints");
    }

    @Override
    public void onWaypointEvent(WaypointManager.WaypointEvent_Type wpEvent, int index, int count) {
        System.out.println("Waypoint moved " + index + "/" + count);
    }

    @Override
    public void onEndWaypointEvent(WaypointManager.WaypointEvent_Type wpEvent) {
        System.out.println("Finish getting waypoints");
    }
}
