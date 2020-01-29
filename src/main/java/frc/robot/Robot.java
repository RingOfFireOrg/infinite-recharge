/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Robot extends TimedRobot {

  Joystick rightstick = new Joystick(0);
  Joystick leftstick = new Joystick(1);
  Joystick manipulatorStick = new Joystick(2);

  AHRS ahrs;

  boolean foundVisionTarget = false;

  DifferentialDrive drive;
  VictorSPX intake, indexer;


  @Override
  public void robotInit() {
    ahrs = new AHRS(SerialPort.Port.kUSB);
    ahrs.reset();

    drive = new DifferentialDrive(new TalonSRX(foo), new TalonSRX(foo));
    intake = new VictorSPX(foo);
    indexer = new VictorSPX(foo2);
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
    drive.arcadeDrive(-leftstick.getY(), -rightstick.getY());
    motorPower = manipulatorStick.getY();
    intake.set(motorPower);
    indexer.set(motorPower);
  }

  @Override
  public void testPeriodic() {
  }
}
