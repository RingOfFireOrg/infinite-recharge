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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;


public class Robot extends TimedRobot {
    double DEAD_ZONE_START = -0.2;
    double DEAD_ZONE_END = 0.2;
    double STOPPED = 0.0;
    double FULL_MULTIPLIER = 1.0;
    int BTN_60_PCT = 6;
    int BTN_70_PCT = 7;
    int BTN_FULL = 8;
    int BTN_INTAKE = 8;

    Joystick rightstick = new Joystick(0);
    Joystick leftstick = new Joystick(1);
    Joystick manipulatorStick = new Joystick(2);
    PWMVictorSPX transfermotor = new PWMVictorSPX(RobotMap.PWM_TRANSFER);
    CANSparkMax intakemotor = new CANSparkMax (RobotMap.CAN_INTAKE, MotorType.kBrushless);

    double speedMultiplier = FULL_MULTIPLIER;

   // CANSparkMax shooterMotor = new CANSparkMax(RobotMap.MOTOR_SHOOTER, MotorType.kBrushless);
   // CANSparkMax shooterMotor2 = new CANSparkMax(RobotMap.MOTOR_SHOOTER2, MotorType.kBrushless);
    
    //AHRS ahrs;

    //NeoTankDrive neoDrive;
    DifferentialDrive tankDrive;

    //Vision vision;

    double tx;
    double ty;
    double ta;


    @Override
    public void robotInit() {
        //ahrs = new AHRS(SerialPort.Port.kUSB);
        //ahrs.reset();

        //neoDrive = new NeoTankDrive();
        tankDrive = new SparkMaxTank ();

      
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
        double shooterSpeed = manipulatorStick.getY();

        boolean Forward_Transfer = manipulatorStick.getRawButton(RobotMap.TRANSFER_FORWARD_BTN);
        boolean Backward_Transfer = manipulatorStick.getRawButton(RobotMap.TRANSFER_BACKWARD_BTN);
        boolean Forward_Intake = manipulatorStick.getRawButton(RobotMap.INTAKE_INPUT_BTN);
        boolean Backward_Intake = manipulatorStick.getRawButton(RobotMap.INTAKE_OUTPUT_BTN);

        if (shooterSpeed > DEAD_ZONE_START && shooterSpeed < DEAD_ZONE_END) shooterSpeed = STOPPED; 

        if (manipulatorStick.getRawButtonPressed(BTN_60_PCT)) {
            speedMultiplier = 0.56;
        } else if (manipulatorStick.getRawButtonPressed(BTN_70_PCT)) {
            speedMultiplier = 0.5635;
        } else if (manipulatorStick.getRawButtonPressed(BTN_FULL)) {
            speedMultiplier = FULL_MULTIPLIER;
        }

        if (Forward_Transfer || Backward_Transfer){

            if (Forward_Transfer) {
                transfermotor.set(0.5);
            } else if (Backward_Transfer) {
                transfermotor.set(-0.5);
            } 
        } else {
            transfermotor.set(0);
        }
        if (Forward_Intake || Backward_Intake){

            if (Forward_Intake) {
                intakemotor.set(0.5);
            } else if (Backward_Intake) {
                intakemotor.set(-0.5);
            } 
        } else {
            intakemotor.set(0);
        }
      

        leftSpeed = leftSpeed * 1;
        rightSpeed = rightSpeed * 1;
        //neoDrive.drive(rightSpeed, leftSpeed, 1.0, true);
        tankDrive.tankDrive(leftSpeed, rightSpeed, true);
        SmartDashboard.putNumber("leftSpeed", leftSpeed);
        SmartDashboard.putNumber("rightSpeed", rightSpeed);

        //shooterMotor.set(shooterSpeed * speedMultiplier);
        //shooterMotor2.set(shooterSpeed * speedMultiplier);
        SmartDashboard.putNumber("Shooter speed", shooterSpeed * speedMultiplier);
        //vision.updateVisionVals();
        //vision.getTargetDistance();


// Creates UsbCamera and MjpegServer [1] and connects them
//CameraServer.getInstance().startAutomaticCapture();

// Creates the CvSink and connects it to the UsbCamera
//CvSink cvSink = CameraServer.getInstance().getVideo();

// Creates the CvSource and MjpegServer [2] and connects them
//CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

    }

        

    @Override
    public void testPeriodic() {
    }
}
