package frc.robot;

public class PID {
    double kP;
    double kI;
    double kD;
    double lastError;
    double integral;
    double minimumOutputValue;
    double maximumOutputValue;
    double pidOutput;
    double error;
    boolean velocityControlMode = false;
    double lastTarget = 0;
    double target = 0;
    double targetMargin = 1;

    double integralMinimumLimit = 0;
    double integralMaximumLimit = 0;
    boolean integralLimitEnabled = false;
     
    public PID (double P, double I, double D) {
        kP = P;
        kI = I;
        kD = D;
        lastError = 0;
        integral = 0;
        minimumOutputValue = -1;
        maximumOutputValue = 1;
        pidOutput = 0;
    }

    public void setVelocityControl(boolean velocityControl) {
        velocityControlMode = velocityControl;
    }

    //only if the target will be rapidly changing
    public void setTargetMargin(double newTargetMargin) {
        targetMargin = newTargetMargin;
    }

    public void setOutputRange(double minimum, double maximum) {
        minimumOutputValue = minimum;
        maximumOutputValue = maximum;
    }

    public void setKP (double P) {
        kP = P;
    }

    public void setKI (double I) {
        kI = I;
    }
    public void setKD (double D) {
        kD = D;
    }

    public void setError(double newError) {
        error = newError;
    }

    public void setError(double newError, double target) {
        error = newError;
        if (Math.abs(target - lastTarget) > targetMargin) {
            integral = 0;
            lastError = 0;
        }
    }

    public void disableIntegralLimit() {
        integralLimitEnabled = false;
    }

    public void setIntegralLimits(double integralMinimumLimit, double integralMaximumLimit) {
        integralLimitEnabled = true;
        this.integralMinimumLimit = integralMinimumLimit;
        this.integralMaximumLimit = integralMaximumLimit;
    }

    public void reset() {
        pidOutput = 0;
        integral = 0;
        lastError = 0;
    }
    
    public void update() {
        integral += (error);
        if (integralLimitEnabled && integral < integralMinimumLimit) {
            pidOutput = (error * kP) + (integralMinimumLimit * kI) + ((error - lastError) * kD);
        } else if (integralLimitEnabled && integral > integralMaximumLimit) {
            pidOutput = (error * kP) + (integralMaximumLimit * kI) + ((error - lastError) * kD);
        } else {
            pidOutput = (error * kP) + (integral * kI) + ((error - lastError) * kD);
        }
        lastError = error;
    }

    public double getOutput() {
        if (pidOutput < minimumOutputValue) {
            return minimumOutputValue;
        } else if (pidOutput > maximumOutputValue) {
            return maximumOutputValue;
        } else {
            return pidOutput;
        }
    }
}