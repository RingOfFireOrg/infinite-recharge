package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import  edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class CimTank extends DifferentialDrive {

        public CimTank () 
        {

            super (new SpeedControllerGroup(
                       new PWMVictorSPX(RobotMap.MOTOR_LEFT)
                       ),   
                   new SpeedControllerGroup(
                       new PWMVictorSPX(RobotMap.MOTOR_RIGHT)
                        )
            );

        }



}