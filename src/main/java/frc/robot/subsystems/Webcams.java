package frc.robot.subsystems;

import frc.robot.RobotMap;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Webcams extends InternalSubsystem {

    public enum CameraStates {
        CAMERA1, CAMERA2
    }

    private CameraStates state;
    private UsbCamera transferCamera;
    private UsbCamera intakeCamera;
    private NetworkTableEntry cameraSelection;

    public Webcams() {
        state = CameraStates.CAMERA1;
        transferCamera = CameraServer.getInstance().startAutomaticCapture(RobotMap.TRANSFER_CAMERA);
        intakeCamera = CameraServer.getInstance().startAutomaticCapture(RobotMap.INTAKE_CAMERA);
        cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    }

    public void setState(CameraStates state) {
        this.state = state;
    }

    public void teleopControl() {
        //Takes driver input and sets states
        if (super.controlSystem.switchCameraViewManipulator.get() || super.controlSystem.switchCameraViewDriver.get()) {
            if (state == CameraStates.CAMERA2){
             state = CameraStates.CAMERA1;
            } else if (state == CameraStates.CAMERA1) {
             state = CameraStates.CAMERA2;
            }
        }
    }

    public void periodic() {
        //this method will be run every code loop
        if (state == CameraStates.CAMERA1) { 
            cameraSelection.setString(transferCamera.getName());
        } else if (state == CameraStates.CAMERA2) {
            cameraSelection.setString(intakeCamera.getName());
        }

    }
}