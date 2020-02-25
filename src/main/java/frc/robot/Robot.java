/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {

  RobotContainer container;
 
  ControlSystems controlSystem;

  AHRS ahrs;
  Vision vision;
  PID visionLineupPid;

  double currentGyroAngle;
  double drivetrainRotationMagnitude;
  

  boolean foundVisionTarget = false;

  @Override
  public void robotInit() {
    controlSystem = ControlSystems.getInstance();
    container = RobotContainer.getInstance();
    ahrs = new AHRS(SerialPort.Port.kUSB);
    ahrs.reset();

    visionLineupPid = new PID(0.02, 0.005, 0);
    visionLineupPid.setOutputRange(-0.5, 0.5);

    vision = new Vision();
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
    container.
    container.robotUpdateSystems();
  }

  @Override
  public void testPeriodic() {
  }
  
}
