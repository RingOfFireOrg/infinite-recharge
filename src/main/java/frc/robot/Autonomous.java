package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.RamseteCommand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter.shooterStates;


public class Autonomous {

    private RobotContainer robotContainer = RobotContainer.getInstance();

    int autonomousStep = 0;
    Timer autonomousTimer;
    double transitionTime = 0;
    PID drive;

    private final String SimpleAuto1 = "SimpleAuto1";
    private final String SimpleAuto2 = "SimpleAuto2";

    private final SendableChooser<String> autonomousChooser = new SendableChooser<>();

    enum AutonomousModes {
        FAR_RIGHT_SHOT, CENTERED_SHOT, FAR_RIGHT_SHOT_COLLECT_SHOT, CENTERED_SHOT_COLLECT_SHOT
    }

    public Autonomous() {
        autonomousTimer = new Timer();
        drive = new PID(0.03, 0, 0);
        autonomousStep = 0;
        autonomousChooser.setDefaultOption(SimpleAuto1, SimpleAuto1);
        autonomousChooser.addOption(SimpleAuto2, SimpleAuto2);

    }

    public boolean runAutonomous() {
        if (autonomousChooser.getSelected() == SimpleAuto1) {
            simpleAuto1();
        } else if (autonomousChooser.getSelected() == SimpleAuto2) {
            simpleAuto2();
        }
        return true;
    }

    //simple auto that will shoot immediately
    public void simpleAuto2() {
        switch (autonomousStep) {
            case 0:
                robotContainer.shooter.setLowerShooterSpeed(1);
                robotContainer.shooter.setShooterSpeed(1);
                robotContainer.shooter.setLowerShooterState(shooterStates.BASE_SPEED);
                robotContainer.shooter.setState(shooterStates.BASE_SPEED);
                if (autonomousTimer.get() - transitionTime > 1000) {
                    switchStep();
                }
                break;
            case 1:
                robotContainer.indexer.setState(Indexer.IndexerState.FORWARD);
                if (autonomousTimer.get() - transitionTime > 4000) {
                    switchStep();
                }
                break;
            case 2:
                robotContainer.indexer.setState(Indexer.IndexerState.IDLE);
                robotContainer.shooter.setLowerShooterState(shooterStates.OFF);
                robotContainer.shooter.setState(shooterStates.OFF);
                break;
        }
    }

    //simple auto that will drive forward for x time and then shoot against wall
    public void simpleAuto1() {
        switch (autonomousStep) {
            case 0:
                drive.setError(-robotContainer.ahrs.getAngle());
                drive.update();
                robotContainer.drive.setDriveSpeeds(0.3 + drive.getOutput(), 0.3 - drive.getOutput());
                if (robotContainer.drive.getLeftInches() > 36/*autonomousTimer.get() - transitionTime > 1000*/) {
                    switchStep();
                }
                break;
            case 1:
                robotContainer.drive.setDriveSpeeds(0, 0);
                if (autonomousTimer.get() - transitionTime > 500) {
                    switchStep();
                }
                break;
            case 2:
                robotContainer.shooter.setLowerShooterSpeed(1);
                robotContainer.shooter.setShooterSpeed(1);
                robotContainer.shooter.setLowerShooterState(shooterStates.BASE_SPEED);
                robotContainer.shooter.setState(shooterStates.BASE_SPEED);
                if (autonomousTimer.get() - transitionTime > 1000) {
                    switchStep();
                }
                break;
            case 3:
                robotContainer.indexer.setState(Indexer.IndexerState.FORWARD);
                if (autonomousTimer.get() - transitionTime > 4000) {
                    switchStep();
                }
                break;
            case 4:
                robotContainer.indexer.setState(Indexer.IndexerState.IDLE);
                robotContainer.shooter.setLowerShooterState(shooterStates.OFF);
                robotContainer.shooter.setState(shooterStates.OFF);
                break;

                
        }
    }

    private void switchStep() {
        autonomousStep ++;
        transitionTime = autonomousTimer.get();
    }

    // public void getAutonomousCommand() {
    //     TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
    //     config.setKinematics(robotContainer.drive.getKinematics());

    //     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //         Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
    //         config
    //     );
    // }

    // RamseteCommand command = new RamseteCommand {
        
    // }
}