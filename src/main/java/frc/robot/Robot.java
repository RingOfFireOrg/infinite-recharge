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
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {

  Joystick rightstick = new Joystick(0);
  Joystick leftstick = new Joystick(1);
  Joystick manipulatorStick = new Joystick(2);
  JoystickButton manipulatorStickButton = new JoystickButton(manipulatorStick, 11);

  AHRS ahrs;
  NeoTankDrive neoDrive;
  Vision vision;
  PID visionLineupPid;

  double currentGyroAngle;
  double drivetrainRotationMagnitude;

  public CANSparkMax frontLeftMotor;
  public CANSparkMax frontRightMotor;
  public CANSparkMax backRightMotor;
  public CANSparkMax backLeftMotor;
  public PWMVictorSPX climberUpMotor;
  public PWMVictorSPX winchMotor;

  boolean foundVisionTarget = false;

  @Override
  public void robotInit() {
    ahrs = new AHRS(SerialPort.Port.kUSB);
    ahrs.reset();

    neoDrive = new NeoTankDrive();

    visionLineupPid = new PID(0.02, 0.005, 0);
    visionLineupPid.setOutputRange(-0.5, 0.5);

    vision = new Vision();
    frontLeftMotor = new CANSparkMax(RobotMap.NEO_FRONT_LEFT, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(RobotMap.NEO_FRONT_RIGHT, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(RobotMap.NEO_BACK_RIGHT, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(RobotMap.NEO_BACK_LEFT, MotorType.kBrushless);
    climberUpMotor = new PWMVictorSPX(RobotMap.CLIMBER_UP);
    winchMotor = new PWMVictorSPX(RobotMap.WINCH);
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
    double manipulatorStickSpeed = manipulatorStick.getY();
    // double manipulatorWinchStickSpeed = manipulatorStickButton.get();
    currentGyroAngle = ahrs.getAngle();
    climberUpMotor.set(manipulatorStickSpeed);// This controls the wheel climber up and down motion
    // WHile button 11 is pushed the winch motor runs
    while (manipulatorStickButton.get()) {
      winchMotor.set(0.5);// This controls the winch motor, so that we can lift the bot
    }

    vision.updateVisionVals();
    vision.getVisionTargetDistance();

    if (vision.foundTarget()) {
      if (!foundVisionTarget) {
        foundVisionTarget = true;
        visionLineupPid.reset();
      }
      visionLineupPid.setError(Math.abs(vision.getVisionTargetAngle()));
      visionLineupPid.update();
      drivetrainRotationMagnitude = visionLineupPid.getOutput();
    }
  }

  @Override
  public void testPeriodic() {
  }
}
