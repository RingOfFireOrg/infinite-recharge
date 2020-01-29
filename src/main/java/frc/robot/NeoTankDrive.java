package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class NeoTankDrive {
    SpeedControllerGroup rightMotors = new SpeedControllerGroup(
            new CANSparkMax(RobotMap.NEO_FRONT_RIGHT, MotorType.kBrushless),
            new CANSparkMax(RobotMap.NEO_BACK_RIGHT, MotorType.kBrushless));
    SpeedControllerGroup leftMotors = new SpeedControllerGroup(
            new CANSparkMax(RobotMap.NEO_FRONT_LEFT, MotorType.kBrushless),
            new CANSparkMax(RobotMap.NEO_BACK_LEFT, MotorType.kBrushless));


    public void drive(double rightSpeed, double leftSpeed, double limiter, boolean isSquared) {
        if (isSquared) {
            leftSpeed = limiter * Math.copySign(leftSpeed * leftSpeed, leftSpeed);
            rightSpeed = limiter * Math.copySign(rightSpeed * rightSpeed, rightSpeed);
        }

        rightMotors.set(limiter * rightSpeed);
        leftMotors.set(limiter * -leftSpeed);
    }

    public void setSpeed(double rightSpeed, double leftSpeed) {
        rightMotors.set(rightSpeed);
        leftMotors.set(leftSpeed);
    }

    public void stop() {
        rightMotors.set(0);
        leftMotors.set(0);
    }
}