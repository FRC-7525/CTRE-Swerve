package frc.robot.pioneersLib.faultCheck;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import frc.robot.pioneersLib.misc.Elastic;
import frc.robot.pioneersLib.misc.Elastic.ElasticNotification;
import frc.robot.pioneersLib.misc.Elastic.ElasticNotification.NotificationLevel;

public class FaultManager {
    public static AtomicReference<FaultManager> instance = new AtomicReference<>();
    private List<Controller> motorControllers = new ArrayList<>();

    public class Controller {
        public MotorController controller;
        public DeviceTypes type;
        public List<String> faults = new ArrayList<>();
        public String deviceName;

        public Controller(String name, DeviceTypes type, MotorController controller) {
            this.controller = controller;
            this.type = type;
            this.deviceName = name;
        }

        public boolean getFault() {
            return faults.size() > 0;
        }

        public MotorController getController()  {
            return controller;
        }

        public String getDevicName() {
            return deviceName;
        }

        public DeviceTypes getDeviceType() {
            return type;
        }

        public void addFault(String fault) {
            faults.add(fault);
        }

        public void removeFault(String fault) {
            faults.remove(fault);
        }
    }

    private FaultManager() {

    }

    public static FaultManager getInstance() {
        if (instance.get() == null) {
            instance.set(new FaultManager());
        } 
        return instance.get();
    }

    public void periodic() {
        checkDevices();
        logDevices();
    }

    public void checkDevices() {
        for (Controller controller : motorControllers) {
            switch (controller.getDeviceType()) {
                case TALON:
                    TalonFX talon = (TalonFX) controller.getController();
                    if (talon.getDeviceTemp().getValueAsDouble() > 100) {
                        controller.addFault("Device Temp Eroneous");
                    } else {
                        controller.removeFault("Device Temp Eroneous");
                    }

                    if (talon.getDeviceID() < 0 || talon.getDeviceID() > 63) {
                        controller.addFault("Device ID Eroneous");
                    } else {
                        controller.removeFault("Device ID Eroneous");
                    }
                    if (talon.getFaultField().getValueAsDouble() != 0) {
                        controller.addFault("Faults Detected");
                    } else {
                        controller.removeFault("Faults Detected");
                    }
                    break;
                // case SPARK:
                //     CANSparkMax spark = (CANSparkMax) controller.getController();
                //     if (spark.getMotorTemperature() > 100) {
                //         controller.addFault("Device Temp Eroneous");
                //     } else {
                //         controller.removeFault("Device Temp Eroneous");
                //     }

                //     if (spark.getDeviceId() < 0 || spark.getDeviceId() > 100) {
                //         controller.addFault("Device ID Eroneous");
                //     } else {
                //         controller.removeFault("Device ID Eroneous");
                //     }

                //     if (spark.getFaults() != 0) {
                //         controller.addFault("Faults Detected");
                //     } else {
                //         controller.removeFault("Faults Detected");
                //     }

                //     break;
                default:
                    break;
            }
        }  
    }

    public void logDevices() {
        for (Controller controller : motorControllers) {
            if (controller.getFault()) {
                for (String fault : controller.faults) {
                    Elastic.sendAlert(
                        new ElasticNotification(
                            NotificationLevel.ERROR, controller.getDevicName(), fault)
                    );
                }
            }
        }
    }

    public void addDevice(Controller controller) {
        // if (!Constants.TESTING) return;
        motorControllers.add(controller);
    }
}
