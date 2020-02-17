package frc.robot;

import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

    private Drivetrain drive = new Drivetrain();

    public void robotUpdateSystems() {
        drive.periodic();
    }
}