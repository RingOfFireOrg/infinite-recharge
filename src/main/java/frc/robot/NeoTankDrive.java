package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class NeoTankDrive extends DifferentialDrive {
    SpeedControllerGroup rightMotors = new SpeedControllerGroup(
        new CANSparkMax(RobotMap.NEO_FRONT_RIGHT, MotorType.kBrushless),
        new CANSparkMax(RobotMap.NEO_BACK_RIGHT, MotorType.kBrushless));
    SpeedControllerGroup leftMotors = new SpeedControllerGroup(
        new CANSparkMax(RobotMap.NEO_FRONT_LEFT, MotorType.kBrushless),
        new CANSparkMax(RobotMap.NEO_BACK_LEFT, MotorType.kBrushless));


    public void drive(double rightSpeed, double leftSpeed, double limiter, boolean isSquared){
        if (isSquared){
            leftSpeed = limiter * Math.copySign(leftSpeed * leftSpeed, leftSpeed);
            rightSpeed = limiter * Math.copySign(rightSpeed * rightSpeed, rightSpeed);
        }

        rightMotors.set(limiter * rightSpeed);
        leftMotors.set(limiter * -leftSpeed);
    }
}