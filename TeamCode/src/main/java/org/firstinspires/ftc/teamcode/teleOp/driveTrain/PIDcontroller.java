package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDcontroller {
    private double kp;
    private double ki;
    private double kd;

    private double target;
    private double integral;
    private double previousError;
    private double previousTime; // Using System.nanoTime() or ElapsedTime for more accurate timing

    public PIDcontroller(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        target = 0.0;
        integral = 0.0;
        previousError = 0.0;
        previousTime = 0.0; // Initialize with current time in actual implementation
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