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
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.RobotMap;

//drive train is tank drive style

public class Drivetrain extends InternalSubsystem{
    CANSparkMax leftMaster = new CANSparkMax(RobotMap.NEO_FRONT_LEFT, MotorType.kBrushless);
    CANSparkMax rightMaster = new CANSparkMax(RobotMap.NEO_FRONT_RIGHT, MotorType.kBrushless);

    CANSparkMax leftSlave = new CANSparkMax(RobotMap.NEO_BACK_LEFT, MotorType.kBrushless);
    CANSparkMax rightSlave = new CANSparkMax(RobotMap.NEO_BACK_RIGHT, MotorType.kBrushless);

    AHRS gyro = new AHRS(SerialPort.Port.kUSB);
    
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(RobotMap.ROBOT_TRACK_WIDTH_IN));
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(RobotMap.DRIVEBOX_KS_CONSTANT, RobotMap.DRIVEBOX_KV_CONSTANT, RobotMap.DRIVEBOX_KA_CONSTANT);
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

    @Override
    public void periodic() {
        odometry.update(getHeading(), leftMaster.getEncoder().getPosition() / RobotMap.DRIVEBASE_GEAR_RATIO * Math.PI * Units.inchesToMeters(6.0),
        rightMaster.getEncoder().getPosition() / RobotMap.DRIVEBASE_GEAR_RATIO * Math.PI * Units.inchesToMeters(6.0));
    }
}