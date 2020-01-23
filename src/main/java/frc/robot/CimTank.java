package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


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