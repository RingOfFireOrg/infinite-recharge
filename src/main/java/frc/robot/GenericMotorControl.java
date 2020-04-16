package frc.robot;

public interface GenericMotorControl {

    public abstract void readInputs ();

    public abstract void activateMotorSpeed ();

    public abstract double getMotorSpeed ();


}