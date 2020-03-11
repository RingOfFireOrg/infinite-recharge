package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.subsystems.*;

public class RobotContainer {

    //mechanical subsystems
    public Drivetrain drive;
    public Intake intake;
    public Shooter shooter; 
    public Climber climber; 
    public Indexer indexer; 
    public ControlPanel controlPanel;
    public Webcams webcams; 

    //other portions
    public AHRS ahrs;
    

    private static RobotContainer robotContainer;

    protected RobotContainer() {
         ahrs = new AHRS(SerialPort.Port.kUSB);
         ahrs.reset();
         
        drive = new Drivetrain(ahrs);
        intake = new Intake();
        shooter = new Shooter();
        climber = new Climber();
        indexer = new Indexer();
        controlPanel = new ControlPanel();
        webcams = new Webcams();
    }

    public static RobotContainer getInstance() {
        if (robotContainer == null) {
            robotContainer = new RobotContainer();
        }
        return robotContainer;
    }

    public void runTeleopControls() {
        drive.teleopControl();
        climber.teleopControl();
        controlPanel.teleopControl();
        webcams.teleopControl();
        runPowerCellSystem();
    }

    public void runPowerCellSystem() {
        intake.teleopControl();
        shooter.teleopControl();
        indexer.teleopControl();
    }

    public void robotUpdateSystems() {
        drive.periodic();
        intake.periodic();
        shooter.periodic();
        climber.periodic();
        indexer.periodic();
        controlPanel.periodic();
        webcams.periodic();
        
    }

    public void runSelectSystem() {
        drive.teleopControl();
        drive.periodic();
    }

    public double getGyroAngle() {
        return ahrs.getAngle();
    }
}