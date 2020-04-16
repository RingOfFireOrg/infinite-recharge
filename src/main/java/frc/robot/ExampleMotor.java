package frc.robot;

public interface ExampleMotor {

    public abstract void spin (double speedVector);

    public abstract void stop ();

    public abstract void setSensitivity (double sensitivity);

    public abstract double getSensitivity ();

}