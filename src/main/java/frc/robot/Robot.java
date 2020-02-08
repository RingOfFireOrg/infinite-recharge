/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;


public class Robot extends TimedRobot {
    CimTank tankDrive;
    ColorSensorV3 colorSensor;
    String currentColor = "none";
    String lastColor = "none";
    int spinCount = 0;

    Joystick rightStick = new Joystick(0);
    Joystick leftStick = new Joystick(1);

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorMatch colorMatcher = new ColorMatch();
    VictorSP diskSpinnerMotor = new VictorSP(9);

    private final Color redTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color greenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color blueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color yellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);


    @Override
    public void robotInit() {
        tankDrive = new CimTank();
        colorSensor = new ColorSensorV3(i2cPort);
        spinCount = 0;
//        colorSensor = new ColorSensorV3(I2C.Port.kMXP);

        colorMatcher.addColorMatch(redTarget);
        colorMatcher.addColorMatch(greenTarget);
        colorMatcher.addColorMatch(blueTarget);
        colorMatcher.addColorMatch(yellowTarget);
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
        double rightSpeed = -rightStick.getY();
        double leftSpeed = -leftStick.getY();
        Color currentDetectedColor = colorSensor.getColor();
        ColorMatchResult colorMatcherResult = colorMatcher.matchClosestColor(currentDetectedColor);

        tankDrive.drive(leftSpeed, rightSpeed, true, 1);

        if (colorMatcherResult.color == redTarget) {
            currentColor = "Red";
            lastColor = currentColor;
        } else if (colorMatcherResult.color == greenTarget) {
            currentColor = "Green";
            lastColor = currentColor;
        } else if (colorMatcherResult.color == blueTarget) {
            currentColor = "Blue";
            lastColor = currentColor;
        } else if (colorMatcherResult.color == yellowTarget) {
            currentColor = "Yellow";
            lastColor = currentColor;
        } else {
            currentColor = "Unknown";
            lastColor = "none";
        }

        if (!currentColor.equals(lastColor)) {
            spinCount += 1;
        }

        if (spinCount > 16){
//            diskSpinnerMotor.set(0);
            SmartDashboard.putString("Target status", "found target");
            SmartDashboard.putNumber("Spin count", spinCount);
        } else {
//            diskSpinnerMotor.set(0.25);
            SmartDashboard.putString("Target status", "no target found");
            SmartDashboard.putNumber("Spin count", spinCount);
        }

        SmartDashboard.putNumber("Red", colorSensor.getRed());
        SmartDashboard.putNumber("Green", colorSensor.getGreen());
        SmartDashboard.putNumber("Blue", colorSensor.getBlue());

        SmartDashboard.putString("ColorMatch detected color", currentColor);
        SmartDashboard.putNumber("ColorMatch confidence", colorMatcherResult.confidence);
    }

    @Override
    public void testPeriodic() {
    }
}
