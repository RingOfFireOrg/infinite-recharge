/*package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;


public class Vision {
    private double ts;
    private double tv;
    private double tx;
    private double ty;
    private double ta;
    private double thor;
    private double tvert;
    AHRS ahrs;

    final int ALLOWED_OFFSET = 20;

    // These numbers must be tuned for your Robot! Be careful!
    final double STEER_K = 0.03; // how hard to drive sideways to center on the target
    final double DRIVE_K = 0.7; // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall

    final double TARGET_HEIGHT = 54.0; // Target's height in centimeters
    final double CAMERA_HEIGHT = 35.0; // Limelight's height in centimeters
    final double CAMERA_MOUNT_ANGLE = 0.0; // Limelight's mounting angle in degrees


    private boolean validTarget() {
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
        thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
        tvert = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);

        if ((tv == 1) && (thor > tvert)) {
            // if there is a target at all and the target (two targets) is wider than it is
            // tall
            return true;
        } else {
            return false;
        }
    }

    boolean hatchPickupReady() {
        if (validTarget()) {
            return true;
        } else {
            return false;
        }
    }

    String leftRightStop = "no value";
    void leftRightAlignmentTest() {
        
        double strafeRightLeft = tx * STEER_K * -1;
        final double MAX_SPEED = 0.4;
        final double MIN_SPEED = 0.1; // stall speed??

        if (Math.abs(tx) > 3) { // three is a random placeholder
            // RJC is it imprtant that the 3 placeholder here and the 3 placeholder above
            // have the same value? If so maybe move to a constant and use that? if not this
            // is fine.

            // deal with min and max speeds for right left
            if (Math.abs(strafeRightLeft) < MIN_SPEED) {
                if (strafeRightLeft > 0) {
                    strafeRightLeft = MIN_SPEED;
                    leftRightStop = "left";
                } else { // if (strafeRightLeft <= 0
                    strafeRightLeft = -MIN_SPEED;
                    leftRightStop = "right";
                }
            } else if (Math.abs(strafeRightLeft) > MAX_SPEED) {
                if (strafeRightLeft > 0) {
                    strafeRightLeft = MAX_SPEED;
                    leftRightStop = "left";
                } else {
                    strafeRightLeft = -MAX_SPEED;
                    leftRightStop = "right";
                }
            }
        } else {
            strafeRightLeft = 0;
            leftRightStop = "stop";
        }

        
        SmartDashboard.putString("Strafe direction left/right", leftRightStop);

    }

    boolean alignment() {
        double strafeRightLeft = tx * STEER_K * -1;
        double strafeForwardBack = (DESIRED_TARGET_AREA / ta) * DRIVE_K;
        final double MAX_SPEED = 0.4;
        final double MIN_SPEED = 0.1; // stall speed??
        boolean rightLeftAligned = false;
        boolean frontBackAligned = false;
        SmartDashboard.putNumber("translate x", strafeRightLeft);

        if (Math.abs(tx) > 3) { // three is a random placeholder
            // RJC is it imprtant that the 3 placeholder here and the 3 placeholder above
            // have the same value? If so maybe move to a constant and use that? if not this
            // is fine.

            // deal with min and max speeds for right left
            if (Math.abs(strafeRightLeft) < MIN_SPEED) {
                if (strafeRightLeft > 0) {
                    strafeRightLeft = MIN_SPEED;
                } else { // if (strafeRightLeft <= 0
                    strafeRightLeft = -MIN_SPEED;
                }
            } else if (Math.abs(strafeRightLeft) > MAX_SPEED) {
                if (strafeRightLeft > 0) {
                    strafeRightLeft = MAX_SPEED;
                } else {
                    strafeRightLeft = -MAX_SPEED;
                }
            }
        } else {
            strafeRightLeft = 0;
            rightLeftAligned = true;
        }

        if (Math.abs(DESIRED_TARGET_AREA / ta) < 0.9) { // is 0.9 close enough??
            // this would speed up or slow down and go drive_K for the perfect setup
            // change the division to subtraction
            if (strafeForwardBack < MIN_SPEED) {
                // won't ever go backward
                strafeForwardBack = MIN_SPEED;
            } else if (strafeForwardBack > MAX_SPEED) {
                strafeForwardBack = MAX_SPEED;
            }
        } else {
            strafeForwardBack = 0;
            frontBackAligned = true;
        }

        if (frontBackAligned && rightLeftAligned) {
            return true;
        } else {
            return false;
        }

    }

    boolean hatchPickup() {
        return true;
        // this will do things in the future
    }

    boolean hatchScore() {
        // this will do things in the future
        return true;
    }

    public void updateVisionVals(){
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);

        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("ty", ty);
        SmartDashboard.putNumber("ta", ta);
    }

    public void getTargetDistance(){
        double heightDifference = TARGET_HEIGHT - CAMERA_HEIGHT;
        double totalAngle = CAMERA_MOUNT_ANGLE + ty;

        SmartDashboard.putNumber("Height", heightDifference);
        SmartDashboard.putNumber("Angle", totalAngle);

        double targetDistance = heightDifference / (Math.sin(Math.toRadians(totalAngle)));
        SmartDashboard.putNumber("Target Distance", targetDistance);
    }
}*/