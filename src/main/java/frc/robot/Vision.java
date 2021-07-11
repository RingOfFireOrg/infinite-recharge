// package frc.robot;

// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import java.lang.Math;

// //TODO update this code -JTW

// public class Vision {
//     private double ts; // Skew
//     private double tv; // Vaild targets
//     private double tx; // x offset (+-27 deg)
//     private double ty; // y offset (+- 20.5 deg)
//     private double ta; // Area
//     private double thor; // Horizontal sidelength
//     private double tvert; // Vertical sidelength

//     private double lineupError;

//     AHRS ahrs;

//     final int ALLOWED_OFFSET = 20;

//     final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall

//     final double TARGET_HEIGHT = 54.0; // Target's height in centimeters
//     final double CAMERA_HEIGHT = 35.0; // Limelight's height in centimeters
//     final double CAMERA_MOUNT_ANGLE = 0.0; // Limelight's mounting angle in degrees
//     final double MAX_SHOOTING_ANGLE = 45; // Maximum shooting angle in degrees


//     public void updateVisionVals(){
//         tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
//         tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
//         ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
//         ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);

//         SmartDashboard.putNumber("tv", tv);
//         SmartDashboard.putNumber("tx", tx);
//         SmartDashboard.putNumber("ty", ty);
//         SmartDashboard.putNumber("ta", ta);
//     }

//     public void getVisionTargetDistance(){
//         double heightDifference = TARGET_HEIGHT - CAMERA_HEIGHT;
//         double totalAngle = CAMERA_MOUNT_ANGLE + ty;

//         SmartDashboard.putNumber("Height", heightDifference);
//         SmartDashboard.putNumber("Angle", totalAngle);

//         double targetDistance = heightDifference / (Math.sin(Math.toRadians(totalAngle)));
//         SmartDashboard.putNumber("Target Distance", targetDistance);
//     }

//     public boolean foundTarget() {
//         if (tv == 1) {
//             return true;
//         } else {
//             return false;
//         }
//     }

//     public boolean isPossibleShot() {
//         if (ts < MAX_SHOOTING_ANGLE) {
//             return true;
//         } else {
//             return false;
//         }
//     }

//     public double getVisionTargetAngle() {
//         updateVisionVals();
//         return tx;
//     }
// }