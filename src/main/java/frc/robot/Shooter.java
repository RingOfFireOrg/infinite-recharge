package frc.robot;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Shooter {
    SpeedControllerGroup shooterMotors = new SpeedControllerGroup(
        new VictorSP(RobotMap.LEFT_SHOOTER_MOTOR),
        new VictorSP(RobotMap.RIGHT_SHOOTER_MOTOR));

    public void setToShoot(double speed){
        shooterMotors.set(-speed);
    }
}