package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.RobotMap;
import frc.robot.PID;

//drive train is tank drive style


public class Drivetrain extends InternalSubsystem{
    CANSparkMax leftMaster = new CANSparkMax(RobotMap.DT_LEFT_MASTER, MotorType.kBrushless);
    CANSparkMax rightMaster = new CANSparkMax(RobotMap.DT_RIGHT_MASTER, MotorType.kBrushless);

    CANSparkMax leftSlave = new CANSparkMax(RobotMap.DT_LEFT_SLAVE, MotorType.kBrushless);
    CANSparkMax rightSlave = new CANSparkMax(RobotMap.DT_RIGHT_SLAVE, MotorType.kBrushless);

    AHRS gyro = new AHRS(SerialPort.Port.kUSB);
    
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(RobotMap.ROBOT_TRACK_WIDTH_IN));
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

    private double speedLimit = 1;

    private double leftGoalSpeed, rightGoalSpeed;

    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(RobotMap.DRIVEBOX_KS_CONSTANT, RobotMap.DRIVEBOX_KV_CONSTANT, RobotMap.DRIVEBOX_KA_CONSTANT);
    
    PID leftSpeedPID = new PID(9.95, 0, 0);
    PID rightSpeedPID = new PID(9.95, 0, 0);

    public Pose2d pose = new Pose2d();

    public Drivetrain () {
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.setInverted(false);
        rightMaster.setInverted(true);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds( // returns wheel speeds in meters per second
            leftMaster.getEncoder().getVelocity() / RobotMap.DRIVEBASE_GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(3.0) / 60,
            rightMaster.getEncoder().getVelocity() / RobotMap.DRIVEBASE_GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(3.0) / 60
        );
    }

    public boolean setDriveSpeeds(double leftGoalSpeed, double rightGoalSpeed) {
        this.leftGoalSpeed = leftGoalSpeed;
        this.rightGoalSpeed = rightGoalSpeed;
        return true;
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
        double inputSpeed = super.controlSystem.leftDriveStick.getY();
        leftGoalSpeed = Math.copySign(inputSpeed * inputSpeed, inputSpeed);
        inputSpeed = super.controlSystem.rightDriveStick.getY();
        rightGoalSpeed = Math.copySign(inputSpeed * inputSpeed, inputSpeed);
    }

    @Override
    public void periodic() {
        odometry.update(getHeading(), leftMaster.getEncoder().getPosition() / RobotMap.DRIVEBASE_GEAR_RATIO * Math.PI * Units.inchesToMeters(6.0),
        rightMaster.getEncoder().getPosition() / RobotMap.DRIVEBASE_GEAR_RATIO * Math.PI * Units.inchesToMeters(6.0));

        leftMaster.set(leftGoalSpeed);
        rightMaster.set(rightGoalSpeed);
    }
}