/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;



public class Robot extends TimedRobot {

    Joystick rightstick = new Joystick(0);
    Joystick leftstick = new Joystick(1);
    Joystick manipulatorStick = new Joystick(2);
    public XboxController gamepadController;

    //AHRS ahrs;

    //NeoTankDrive neoDrive;
    DifferentialDrive tankDrive;
 
    PWMVictorSPX transferMotor = new PWMVictorSPX(RobotMap.TRANSFER);
    TalonFX testMotor = new TalonFX(1);
    //Vision vision;

    double tx;
    double ty;
    double ta;


    @Override
    public void robotInit() {
        //ahrs = new AHRS(SerialPort.Port.kUSB);
        //ahrs.reset();

        //neoDrive = new NeoTankDrive();
        tankDrive = new CimTank ();
        //vision = new Vision();
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {
        double rightSpeed = rightstick.getY();
        double leftSpeed = leftstick.getY();
        double collectorSpeed = gamepadController.getRawAxis(RobotMap.MANIPULATOR_RIGHT_JOYSTICK_X);

        //neoDrive.drive(rightSpeed, leftSpeed, 1.0, true);
        //TankDrive.tankDrive(leftSpeed, rightSpeed, true);
        testMotor.set(TalonFXControlMode.PercentOutput, collectorSpeed * .5);

        //vision.updateVisionVals();
        //vision.getTargetDistance();
    }

    public void testPeriodic() {
    }
}
