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

import edu.wpi.first.wpilibj.XboxController;



public class Robot extends TimedRobot {

    Joystick rightstick = new Joystick(0);
    Joystick leftstick = new Joystick(1);
    Joystick manipulatorStick = new Joystick(2);
    public XboxController gamepadController = new XboxController(RobotMap.MANIPULATOR_GAMEPAD);

    //AHRS ahrs;

    //NeoTankDrive neoDrive;
    DifferentialDrive tankDrive;
 
    //Change the motor type here
    FalconExample testMotor = new FalconExample(RobotMap.TESTMOTOR1);

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
        double rightJoystickX = gamepadController.getRawAxis(RobotMap.MANIPULATOR_RIGHT_JOYSTICK_X);

        //TankDrive.tankDrive(leftSpeed, rightSpeed, true);
        testMotor.setSensitivity(0.5);
        testMotor.spin(rightJoystickX);

        //vision.updateVisionVals();
        //vision.getTargetDistance();
    }

    public void testPeriodic() {
    }
}
