package com.dronegcs.mavlink.is.drone.calibration;

import com.dronegcs.mavlink.core.firmware.FirmwareType;
import com.dronegcs.mavlink.is.drone.Drone;
import com.dronegcs.mavlink.is.drone.DroneInterfaces;
import com.dronegcs.mavlink.is.drone.DroneVariable;
import com.dronegcs.mavlink.is.drone.Preferences;
import com.dronegcs.mavlink.is.drone.parameters.Parameter;
import com.dronegcs.mavlink.is.protocol.msg_metadata.enums.MAV_EXT_COMPASS_ORIENTATION;
import com.dronegcs.mavlink.is.protocol.msg_metadata.enums.MAV_PROTOCOL_CAPABILITY;
import com.dronegcs.mavlink.is.protocol.msgbuilder.MavLinkCalibration;
import com.dronegcs.mavlink.is.protocol.msgbuilder.MavLinkStreamRates;
import org.springframework.stereotype.Component;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;

import static com.dronegcs.mavlink.is.drone.calibration.CalibrateCompass.CompassType.UNKNOWN;

@Component
public class CalibrateCompass extends DroneVariable implements DroneInterfaces.OnDroneListener, Calibration {

    private CompassType calibrationType = UNKNOWN;

    private int expectedAmount;
    private double averageX;
    private double averageY;
    private double averageZ;

    public enum CompassType {
        PIXHAWK,
        APM_2_5,
        EXTERNAL,

        UNKNOWN,
    }

    class Measurement {
        private final double x;
        private final double y;
        private final double z;

        public Measurement(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            Measurement that = (Measurement) o;
            return Double.compare(Math.round(that.x * 10)/10, Math.round(x * 10)/10) == 0 &&
                    Double.compare(Math.round(that.y * 10)/10, Math.round(y * 10)/10) == 0 &&
                    Double.compare(Math.round(that.z * 10)/10, Math.round(z * 10)/10) == 0;
        }

        @Override
        public int hashCode() {
            return Objects.hash(Math.round(x * 10)/10, Math.round(y * 10)/10, Math.round(z * 10)/10);
        }
    }

    private static final int MEASUREMENT_AMOUNT = 150;

    private boolean calibrating;
    private Preferences.Rates originalRates;
    private Set<Measurement> rawImuSet;
    private int rawImuAmount;

    static int called;
    public void init() {
        if (called++ > 1)
            throw new RuntimeException("Not a Singleton");
        drone.addDroneListener(this);
    }

    public void setType(CompassType type) {
        switch (type) {
            case EXTERNAL:
                setOrientation(MAV_EXT_COMPASS_ORIENTATION.ROTATION_ROLL_180);
                break;
            case APM_2_5:
                setOrientation(MAV_EXT_COMPASS_ORIENTATION.ROTATION_NONE);
                break;
            case PIXHAWK:
//                if (is_Greater_Copter_3_01_or_Plane_2_74){
                setOrientation(MAV_EXT_COMPASS_ORIENTATION.ROTATION_NONE);
//                }
//                else
//                {
//                    setOrientation(MAV_EXT_COMPASS_ORIENTATION.ROTATION_ROLL_180);
//                }
                break;
        }
        this.calibrationType = type;
    }

    public CompassType getType() {
        return calibrationType;
    }

    public void setOrientation(MAV_EXT_COMPASS_ORIENTATION orientation) {
        Parameter parameter = drone.getParameters().getParameter("COMPASS_ORIENT");
        if (parameter == null)
            return;
        parameter.setValue(orientation.ordinal());
        drone.getParameters().sendParameter(parameter);
    }

    public MAV_EXT_COMPASS_ORIENTATION getOrientation() {
        Parameter parameter = drone.getParameters().getParameter("COMPASS_ORIENT");
        if (parameter == null)
            return null;
        return MAV_EXT_COMPASS_ORIENTATION.values()[parameter.getValue().intValue()];
    }

    @Override
    public boolean start() {
        if (getCompassFeatures() == null || !getCompassFeatures().contains(calibrationType))
            return false;

        switch (calibrationType){
            case APM_2_5:
                return startAPM25();
            case PIXHAWK:
                return startPixhawk(true);
            case EXTERNAL:
                return startAPMExternal();
        }
        return false;
    }

    public Set<CompassType> getCompassFeatures() {
        Set<CompassType> supportedTypes = new HashSet<CompassType>();
        supportedTypes.addAll(Arrays.asList(CompassType.values()));

//        if (drone.getFirmwareVersion() > Version.Parse("3.2.1") &&
        if (drone.getFirmwareType().equals(FirmwareType.ARDU_COPTER2)) {
            supportedTypes.remove(CompassType.APM_2_5);
            supportedTypes.remove(CompassType.EXTERNAL);
            supportedTypes.remove(CompassType.PIXHAWK);
            return null;
        }

//        if (drone.getFirmwareVersion() >= Version.Parse("3.7.1") &&
         if (drone.getFirmwareType().equals(FirmwareType.ARDU_PLANE))
        {
            supportedTypes.remove(CompassType.APM_2_5);
            supportedTypes.remove(CompassType.EXTERNAL);
            supportedTypes.remove(CompassType.PIXHAWK);
            return null;
        }

        if (this.drone.getCapabilities() == null || (drone.getCapabilities().getByteArray() & MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION) == 0)
        {
            // Means we cannot do live calibration
            supportedTypes.remove(CompassType.APM_2_5);
            supportedTypes.remove(CompassType.EXTERNAL);
            supportedTypes.remove(CompassType.PIXHAWK);
            return null;
        }

        return supportedTypes;
    }

    private boolean startPixhawk(boolean is_Greater_Copter_3_01_or_Plane_2_74) {
        boolean externalcompass = true;
        if (!is_Greater_Copter_3_01_or_Plane_2_74)
            externalcompass = false;

        selectCompass(externalcompass, true, false, true, false, false, false, true);
        return startLiveCalibration(MEASUREMENT_AMOUNT);
    }

    private boolean startAPM25()
    {
        selectCompass(true, false, false, false, false, false, false, true);
        return startLiveCalibration(MEASUREMENT_AMOUNT);
    }

    private boolean startAPMExternal() {
        selectCompass(true, false,false, true, false, false, false, true);
        return startLiveCalibration(MEASUREMENT_AMOUNT);
    }

    private boolean selectCompass(boolean compassExternal1, boolean compassExternal2, boolean compassExternal3,
                                    boolean compassUse1, boolean compassUse2, boolean compassUse3,
                                    boolean primary, boolean learn) {
        Parameter param = null;

        param = drone.getParameters().getParameter("COMPASS_EXTERNAL");
        param.setValue(compassExternal1 ? 1 : 0);
        drone.getParameters().sendParameter(param);

        param = drone.getParameters().getParameter("COMPASS_EXTERN2");
        param.setValue(compassExternal2 ? 1 : 0);
        drone.getParameters().sendParameter(param);

        param = drone.getParameters().getParameter("COMPASS_EXTERN3");
        param.setValue(compassExternal3 ? 1 : 0);
        drone.getParameters().sendParameter(param);

        param = drone.getParameters().getParameter("COMPASS_USE1");
        param.setValue(compassUse1 ? 1 : 0);
        drone.getParameters().sendParameter(param);

        param = drone.getParameters().getParameter("COMPASS_USE2");
        param.setValue(compassUse2 ? 1 : 0);
        drone.getParameters().sendParameter(param);

        param = drone.getParameters().getParameter("COMPASS_USE3");
        param.setValue(compassUse3 ? 1 : 0);
        drone.getParameters().sendParameter(param);

        param = drone.getParameters().getParameter("COMPASS_PRIMARY");
        param.setValue(primary ? 1 : 0);
        drone.getParameters().sendParameter(param);

        param = drone.getParameters().getParameter("COMPASS_LEARN");
        param.setValue(learn ? 1 : 0);
        drone.getParameters().sendParameter(param);

        return true;
    }

    private boolean startLiveCalibration(int measurement) {
        if(drone.getState().isFlying()) {
            this.calibrating = false;
            return false;
        }

        this.calibrating = true;
        drone.notifyDroneEvent(DroneInterfaces.DroneEventsType.EXT_CALIB_MAGNETOMETER_START);

        this.expectedAmount = measurement;
        this.rawImuSet = new HashSet<>();
        this.rawImuAmount = 0;
        this.originalRates = drone.getPreferences().getRates();
        MavLinkStreamRates.setupStreamRates(drone, 0,
                0, 0, 0, 0, 0, 50, 0);
        MavLinkCalibration.sendStartMagnometerCalibrationMessage(drone);
        return true;
    }

    @Override
    public boolean stop() {
        MavLinkStreamRates.setupStreamRates(drone, originalRates.extendedStatus,
                originalRates.extra1, originalRates.extra2, originalRates.extra3, originalRates.position, originalRates.rcChannels,
                originalRates.rawSensors, originalRates.rawController);
        this.calibrating = false;

        Parameter compassLearnStatus = drone.getParameters().getParameter("COMPASS_LEARN");
        compassLearnStatus.setValue(0);
        drone.getParameters().sendParameter(compassLearnStatus);

        Parameter paramX = drone.getParameters().getParameter("COMPASS_OFS_X");
        paramX.setValue(averageX);
        drone.getParameters().sendParameter(paramX);

        Parameter paramY = drone.getParameters().getParameter("COMPASS_OFS_Y");
        paramY .setValue(averageY);
        drone.getParameters().sendParameter(paramY);

        Parameter paramZ = drone.getParameters().getParameter("COMPASS_OFS_Z");
        paramZ.setValue(averageZ);
        drone.getParameters().sendParameter(paramZ);

        drone.notifyDroneEvent(DroneInterfaces.DroneEventsType.EXT_CALIB_MAGNETOMETER_FINISH);
        drone.getMessegeQueue().push("New Compass Offsets x:" + averageX + " y:" + averageY + " z:" + averageZ);

        return true;
    }

    @Override
    public void onDroneEvent(DroneInterfaces.DroneEventsType event, Drone drone) {
        switch (event) {
            case MAGNETOMETER:
                if (!this.calibrating)
                    return;

                this.rawImuSet.add(new Measurement(drone.getMagnetometer().getXmag(),
                        drone.getMagnetometer().getYmag(),drone.getMagnetometer().getZmag()));
                this.averageX = (this.averageX * this.rawImuAmount + drone.getMagnetometer().getXmag()) / (this.rawImuAmount + 1);
                this.averageY = (this.averageY * this.rawImuAmount + drone.getMagnetometer().getYmag()) / (this.rawImuAmount + 1);
                this.averageZ = (this.averageZ * this.rawImuAmount + drone.getMagnetometer().getZmag()) / (this.rawImuAmount + 1);
                this.rawImuAmount++;
                if (this.rawImuSet.size() == this.expectedAmount) {
                    drone.getMessegeQueue().push("Compass Calibration Finished (" + Math.round(this.rawImuAmount/this.rawImuSet.size()*100.0) + "%)");
                    this.stop();
                }
                else {
                    this.drone.notifyDroneEvent(DroneInterfaces.DroneEventsType.EXT_CALIB_MAGNETOMETER_PROGRESS);
                }
                break;

            default:
                break;
        }
    }

    public void setCalibrating(boolean flag) {
        this.calibrating = flag;
    }

    @Override
    public boolean isCalibrating() {
        return this.calibrating;
    }

    public double[] getCurrentCenter() {
        return new double[]{this.averageX,this.averageY,this.averageY};
    }
}
