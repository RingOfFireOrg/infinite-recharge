package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;


public class Vision {
    private double ts; // Skew
    private double tv; // Valid targets
    public double tx; // x offset (+-27 deg)
    private double ty; // y offset (+- 20.5 deg)
    private double ta; // Area
    private double thor; // Horizontal sidelength
    private double tvert; // Vertical sidelength

    private double lineupError;

    public enum VisionStates {
        PID_RESET, PID_UPDATE, SET_MOTORS, CHECK_LINEUP
    }

    PID visionLineupPid;

    AHRS ahrs = new AHRS(SerialPort.Port.kUSB);

    final int ALLOWED_OFFSET = 20;

    final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall

    final double TARGET_HEIGHT = 54.0; // Target's height in centimeters
    final double CAMERA_HEIGHT = 35.0; // Limelight's height in centimeters
    final double CAMERA_MOUNT_ANGLE = 0.0; // Limelight's mounting angle in degrees
    final double MAX_SHOOTING_ANGLE = 45; // Maximum shooting angle in degrees

    double currentGyroAngle;
    double drivetrainRotationMagnitude;
    boolean lookingForVisionTarget;

    VisionStates currentStep = VisionStates.PID_RESET;

    public void updateVisionVals() {
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);

        SmartDashboard.putNumber("tv", tv);
        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("ty", ty);
        SmartDashboard.putNumber("ta", ta);
    }

    public double getVisionTargetDistance() {
        double heightDifference = TARGET_HEIGHT - CAMERA_HEIGHT;
        double totalAngle = CAMERA_MOUNT_ANGLE + ty;

        SmartDashboard.putNumber("Height", heightDifference);
        SmartDashboard.putNumber("Angle", totalAngle);

        double targetDistance = heightDifference / (Math.sin(Math.toRadians(totalAngle)));
        SmartDashboard.putNumber("Target Distance", targetDistance);

        return targetDistance;
    }

    public boolean foundTarget() {
        if (tv == 1) {
            return true;
        } else {
            return false;
        }
    }

    public boolean linedUp() {
        if (tx < 2) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isPossibleShot() {
        if (ts < MAX_SHOOTING_ANGLE) {
            return true;
        } else {
            return false;
        }
    }

    public double getVisionTargetAngle(double currentGyroAngle) {
        updateVisionVals();
        double visionTargetAngle = (currentGyroAngle - tx) % 360;
        return visionTargetAngle;
    }

    public void writeDistanceAndAngle() {
        SmartDashboard.putNumber("Distance", getVisionTargetDistance());
        SmartDashboard.putNumber("Angle to target", tx);
    }

    public void initVision() {
        boolean lookingForVisionTarget = false;

        visionLineupPid = new PID(0.003, 0.0005, 0);
        visionLineupPid.setOutputRange(-0.2, 0.2);
    }

    public void runVision() {
        double currentTargetDist = getVisionTargetDistance();
        currentGyroAngle = ahrs.getAngle();

        switch (currentStep) {
            case PID_RESET:
                if (!lookingForVisionTarget) {
                    lookingForVisionTarget = true;
                    visionLineupPid.reset();
                }
                currentStep = VisionStates.PID_UPDATE;
            case PID_UPDATE:
                visionLineupPid.setError(getVisionTargetAngle(currentGyroAngle));
                visionLineupPid.update();
                currentStep = VisionStates.SET_MOTORS;
            case SET_MOTORS:
                drivetrainRotationMagnitude = -visionLineupPid.getOutput();
                currentStep = VisionStates.CHECK_LINEUP;
            case CHECK_LINEUP:
                if (Math.abs(tx) > 2) {
//                    neoDrive.setSpeed(drivetrainRotationMagnitude, drivetrainRotationMagnitude);  //TODO Change to correct drivetrain
                    SmartDashboard.putBoolean("Vision status", lookingForVisionTarget);
                    SmartDashboard.putString("Target status", "Going to target!");
                } else {
                    lookingForVisionTarget = false;
                    SmartDashboard.putString("Target status", "Found target!");
                    currentStep = VisionStates.PID_RESET;
                }
        }
    }
}