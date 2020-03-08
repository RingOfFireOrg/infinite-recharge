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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Robot extends TimedRobot {

    Joystick rightstick = new Joystick(0);
    Joystick leftstick = new Joystick(1);
    Joystick manipulatorStick = new Joystick(2);

    //AHRS ahrs;

    //NeoTankDrive neoDrive;
    DifferentialDrive tankDrive;
 
    PWMVictorSPX transferMotor = new PWMVictorSPX(RobotMap.TRANSFER);

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
        double collectorSpeed = manipulatorStick.getY();

        //neoDrive.drive(rightSpeed, leftSpeed, 1.0, true);
        tankDrive.tankDrive(leftSpeed, rightSpeed, true);
        transferMotor.set(collectorSpeed);

        //vision.updateVisionVals();
        //vision.getTargetDistance();
    }

    @Override
    public void testPeriodic() {
    }
}
