package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;


public class Autonomous {

    private RobotContainer robotContainer = RobotContainer.getInstance();

    enum AutonomousModes {
        FAR_RIGHT_SHOT, CENTERED_SHOT, FAR_RIGHT_SHOT_COLLECT_SHOT, CENTERED_SHOT_COLLECT_SHOT
    }

    public boolean runAutonomous() {
        return true;
    }

    public void getAutonomousCommand() {
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
        config.setKinematics(robotContainer.drive.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
            config
        );

        RamseteCommand RamseteCommand = new RamseteCommand (

        )
    }
}