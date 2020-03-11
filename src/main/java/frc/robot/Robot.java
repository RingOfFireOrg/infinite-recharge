/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;


public class Robot extends TimedRobot {

  RobotContainer container;

  ControlSystems controlSystem;

  Vision vision;

  Autonomous autonomous;

  @Override
  public void robotInit() {
    controlSystem = ControlSystems.getInstance();
    container = RobotContainer.getInstance();
    vision = new Vision();
    autonomous = new Autonomous();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    autonomous.runAutonomous();
    container.robotUpdateSystems();
  }

  @Override
  public void teleopPeriodic() {
    container.runTeleopControls();
    container.robotUpdateSystems();
  }

  @Override
  public void testPeriodic() {
    container.drive.setDriveSpeeds(new ChassisSpeeds(0, 1, 0));
    container.robotUpdateSystems();
  }

}