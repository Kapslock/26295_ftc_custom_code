package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Controllers;

import com.acmerobotics.dashboard.config.Config;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID.functionType;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;

@Config
public class PoseController {
    public PID xPID;
    public PID yPID;
    public PID tPID;
    public static double kPX = 10.5;
    public static double kPY = 10.5;
    public static double kPTheta= 5;
    public static double kIX, kIY, kITheta = 0;
    public static double kDX = 0;
    public static double kDY = 0;
    public static double kDTheta = 0;
    private double lastTheta = 0;
    private double lastX;
    private double lastY;
    public PoseController(){
        this.xPID = new PID(kPX, kIX, kDX, functionType.SQRT);
        this.yPID = new PID(kPY, kIY, kDY, functionType.SQRT);
        this.tPID = new PID(kPTheta, kITheta, kDTheta, functionType.SQRT);
    }
    public SimpleMatrix calculate(SimpleMatrix pose, SimpleMatrix desiredPose){
        if (pose.hasUncountable()){
            pose.set(0, 0, lastX);
            pose.set(1, 0, lastY);
            pose.set(2,0,lastTheta);
        } else {
            lastX = pose.get(0, 0);
            lastY = pose.get(1, 0);
            lastTheta = pose.get(2,0);
        }



        SimpleMatrix errorField = new SimpleMatrix(
                new double[][]{
                        new double[]{desiredPose.get(0,0)-pose.get(0,0)},
                        new double[]{desiredPose.get(1,0)-pose.get(1,0)},
                        new double[]{Utils.angleWrap(desiredPose.get(2, 0) - pose.get(2, 0))}
                }
        );
        SimpleMatrix errorRobot = Utils.rotateGlobalToBody(errorField, pose.get(2,0));

        double vX = xPID.calculate(errorRobot.get(0, 0),0);
        double vY = yPID.calculate(errorRobot.get(1, 0),0);
        double omega = tPID.calculate(Utils.angleWrap(desiredPose.get(2, 0) - pose.get(2, 0)),0);

        SimpleMatrix vRobot = new SimpleMatrix (
                new double[][] {
                        new double[]{vX},
                        new double[]{vY},
                        new double[]{omega}
                }
        );
        return Utils.inverseKinematics(vRobot);
    }
}


