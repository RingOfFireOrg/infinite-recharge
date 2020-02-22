package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;



public class SparkMaxTank extends DifferentialDrive {

        public SparkMaxTank () 
        {

            super (new SpeedControllerGroup(
                       new  CANSparkMax(RobotMap.NEO_BACK_LEFT, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless)),
                       //new  CANSparkMax(RobotMap.NEO_BACK_RIGHT, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless)),
                       
                   new SpeedControllerGroup(
                       new  CANSparkMax(RobotMap.NEO_FRONT_RIGHT, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless))
                    );
        }



}