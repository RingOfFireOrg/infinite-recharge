package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class CimTank {
    SpeedControllerGroup leftMotors = new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.MOTOR_LEFT));
    SpeedControllerGroup rightMotors = new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.MOTOR_RIGHT));

    public void drive(double leftSpeed, double rightSpeed, boolean isSquared, double limiter){
        if (isSquared) {
            leftSpeed = Math.copySign(leftSpeed * leftSpeed, leftSpeed);
            rightSpeed = Math.copySign(rightSpeed * rightSpeed, rightSpeed);
        }

        if (Math.abs(leftSpeed) > limiter){
            leftSpeed = limiter;
        }
        if (Math.abs(rightSpeed) > limiter){
            rightSpeed = limiter;
        }

        rightMotors.set((rightSpeed));
        leftMotors.set(leftSpeed);

    }
}
