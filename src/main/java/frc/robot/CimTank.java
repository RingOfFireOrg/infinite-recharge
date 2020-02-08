package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;

public class CimTank {
    SpeedControllerGroup leftMotors = new SpeedControllerGroup(new VictorSP(RobotMap.MOTOR_LEFT));
    SpeedControllerGroup rightMotors = new SpeedControllerGroup(new VictorSP(RobotMap.MOTOR_RIGHT));

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
