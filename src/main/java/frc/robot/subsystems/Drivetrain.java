package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.RobotMap;
import frc.robot.PID;
import frc.robot.Robot;
import frc.robot.RobotContainer;

//drive train is tank drive style


public class Drivetrain extends InternalSubsystem{

    SpeedControllerGroup leftMotors, rightMotors;
    
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(RobotMap.ROBOT_TRACK_WIDTH_IN));
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

    private double leftGoalSpeed, rightGoalSpeed;
    private CANEncoder leftEncoder, rightEncoder;

    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(RobotMap.DRIVEBOX_KS_CONSTANT, RobotMap.DRIVEBOX_KV_CONSTANT, RobotMap.DRIVEBOX_KA_CONSTANT);
    
    PID leftSpeedPID = new PID(9.95, 0, 0);
    PID rightSpeedPID = new PID(9.95, 0, 0);

    public Pose2d pose = new Pose2d();

    CANSparkMax rightForward, rightBack, leftForward, leftBack;

    public Drivetrain () {
        rightForward = new CANSparkMax(RobotMap.DT_RIGHT_FORWARD, MotorType.kBrushless);
        rightBack = new CANSparkMax(RobotMap.DT_RIGHT_BACK, MotorType.kBrushless);
        leftForward = new CANSparkMax(RobotMap.DT_LEFT_FORWARD, MotorType.kBrushless);
        leftBack = new CANSparkMax(RobotMap.DT_LEFT_BACK, MotorType.kBrushless);

        rightForward.setInverted(true);
        rightBack.setInverted(true);
        leftForward.setInverted(false);
        leftBack.setInverted(false);

        leftMotors = new SpeedControllerGroup(leftForward, leftBack);
        rightMotors = new SpeedControllerGroup(rightForward, rightBack);
        leftEncoder = leftForward.getEncoder();
        rightEncoder = rightForward.getEncoder();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-super.robotContainer.getGyroAngle());
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds( // returns wheel speeds in ft per second
            leftEncoder.getVelocity() / RobotMap.DRIVEBASE_GEAR_RATIO * 2 * Math.PI * 3 / 12 / 60,
            rightEncoder.getVelocity() / RobotMap.DRIVEBASE_GEAR_RATIO * 2 * Math.PI * 3 / 12 / 60
        );
    }

    public boolean setRawDriveSpeeds(double leftGoalSpeed, double rightGoalSpeed) {
        this.leftGoalSpeed = leftGoalSpeed;
        this.rightGoalSpeed = rightGoalSpeed;
        return true;
    }

    public void setDriveSpeeds(ChassisSpeeds chassisSpeeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        //setVelocities(Units.metersToFeet(wheelSpeeds.leftMetersPerSecond), Units.metersToFeet(wheelSpeeds.rightMetersPerSecond));
    }
    
    // public double getLeftGoalVelocity() {

    // }
    // public boolean setVelocities(double leftFeetPerSecond, double rightFeetPerSecond) {

    // }

    public double getLeftFeet() {
        return leftEncoder.getPosition() / RobotMap.DRIVEBASE_GEAR_RATIO * Math.PI * RobotMap.DRIVE_WHEEL_DIAMETER_IN / 12;
    }

    public double getRightFeet() {
        return rightEncoder.getPosition() / RobotMap.DRIVEBASE_GEAR_RATIO * Math.PI * RobotMap.DRIVE_WHEEL_DIAMETER_IN / 12;
    }
    
    public SimpleMotorFeedforward getFeedForward() {
        return feedForward;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public PID getLeftPID() {
        return leftSpeedPID;
    }

    public PID getRightPID() {
        return rightSpeedPID;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void teleopControl() {
        double leftInputSpeed = -0.8 * super.controlSystem.leftDriveStick.getY();
        leftGoalSpeed = Math.copySign(leftInputSpeed * leftInputSpeed, leftInputSpeed);
        double rightInputSymbol = -0.8 * super.controlSystem.rightDriveStick.getY();
        rightGoalSpeed = Math.copySign(rightInputSymbol * rightInputSymbol, rightInputSymbol);
    }

    @Override
    public void periodic() {
        odometry.update(getHeading(), leftEncoder.getPosition() / RobotMap.DRIVEBASE_GEAR_RATIO * Math.PI * 6 / 12,
        rightEncoder.getPosition() / RobotMap.DRIVEBASE_GEAR_RATIO * Math.PI * 6 / 12);

        leftMotors.set(leftGoalSpeed);
        rightMotors.set(rightGoalSpeed);
    }
}