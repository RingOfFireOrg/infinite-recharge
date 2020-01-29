package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class NeoTankDrive {
    SpeedControllerGroup leftCluster, rightCluster;
    public NeoTankDrive () {
        leftCluster = new SpeedControllerGroup(
            new CANSparkMax(RobotMap.NEO_FRONT_LEFT, MotorType.kBrushless),
            new CANSparkMax(RobotMap.NEO_BACK_LEFT, MotorType.kBrushless));
        rightCluster = new SpeedControllerGroup(
            new CANSparkMax(RobotMap.NEO_FRONT_RIGHT, MotorType.kBrushless),
            new CANSparkMax(RobotMap.NEO_BACK_RIGHT, MotorType.kBrushless));

        leftCluster.setInverted(true);
        rightCluster.setInverted(false);
    }


    public void drive(double rightSpeed, double leftSpeed, double limiter, boolean isSquared){
        if (isSquared){
            leftSpeed = limiter * Math.copySign(leftSpeed * leftSpeed, leftSpeed);
            rightSpeed = limiter * Math.copySign(rightSpeed * rightSpeed, rightSpeed);
        }

        rightCluster.set(limiter * rightSpeed);
        leftCluster.set(limiter * leftSpeed);
    }
}