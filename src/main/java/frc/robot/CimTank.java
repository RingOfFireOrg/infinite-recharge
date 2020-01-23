package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class CimTank extends DifferentialDrive {

        public CimTank () 
        {

            super (new SpeedControllerGroup(
                       new WPI_TalonSRX(RobotMap.MOTOR_FRONT_LEFT),
                       new WPI_VictorSPX(RobotMap.MOTOR_BACK_LEFT)
                       ),    
                   new SpeedControllerGroup(
                       new WPI_TalonSRX(RobotMap.MOTOR_FRONT_RIGHT), 
                       new WPI_VictorSPX(RobotMap.MOTOR_BACK_RIGHT)
                    )
            );

        }



}