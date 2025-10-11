package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

public class PIDController {
    public double kp;
    public double ki;
    public double kd;

    public double target;
    private double integral;
    private double previousError;
    private double previousTime; // Using System.nanoTime() or ElapsedTime for more accurate timing

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        target = 0.0;
        integral = 0.0;
        previousError = 0.0;
        previousTime = 0.0; // Initialize with current time in actual implementation
    }

    public void setKP(double kp) {
        this.kp = kp;
    }
    public void setKI(double ki) {
        this.ki = ki;
    }
    public void setKD(double kd) {
        this.kd = kd;
    }
    public void setTarget(double target) {
        this.target = target;
    }

    public double calculateOutput(double current, double time) {

        double error = target - current;
        double deltaTime = time - previousTime;

        // Avoid division by zero if deltaTime is very small
        double derivative = (deltaTime > 0) ? (error - previousError) / deltaTime : 0.0;

        integral += error * deltaTime;

        // Calculate the output
        double output = kp * error + ki * integral + kd * derivative;

        previousError = error;
        previousTime = time;

        return output;

    }
}