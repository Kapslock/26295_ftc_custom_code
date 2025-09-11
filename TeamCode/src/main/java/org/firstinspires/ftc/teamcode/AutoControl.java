package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

public class AutoControl {

    private LinearOpMode myOpMode = null;
    public RobotControl robot;
    public GoBildaPinpointDriver odo;
    public EnhancedNavigation navigation;

    // Field coordinates (in mm)


    public double power = 0.6;

    public double skibidi = 1.0;

    public double specArmTarget = -2050;
    public double weird = 0.6;
    private boolean isTeleOp = false;


    private static double cPower = 1;// seconds
    private static final double specArmAdjust = -150;


    public AutoControl (LinearOpMode opmode) {
        myOpMode = opmode;
    }
    public void initialize(){
        robot = new RobotControl(myOpMode);
        robot.init();
        robot.resetEncoders();
        cPower = power;

        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-6.25, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        navigation = new EnhancedNavigation(robot, odo);
    }

    public void setup(){
        robot.armTarget = -1000;
        robot.controllerDrive(0, 1, 0, 1);
    }


    public void updateTelemetry(){
        odo.update();
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(DistanceUnit.MM),
                pos.getY(DistanceUnit.MM),
                pos.getHeading(AngleUnit.DEGREES));
        myOpMode.telemetry.addData("Current", data);
        myOpMode.telemetry.addData("arm pos", robot.armMotor.getCurrentPosition());
        myOpMode.telemetry.addData("target", robot.armTarget);
        myOpMode.telemetry.update();
    }

    public void moveTo(double x, double y, double h, double p, double t, double c){
        navigation.resetController();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        if(isTeleOp){
            t = 5;
        }
        while (myOpMode.opModeIsActive() && timer.seconds() < t) {
            if(isTeleOp){
                if(myOpMode.gamepad1.right_stick_button){
                    break;
                }
            }
            if (navigation.navigateToPosition(x, y, h, p)) {
                break;
            }
            updateTelemetry();
            robot.armControl(0, c);
        }
        robot.resetDrive();
    }

    public void wait(double t) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (myOpMode.opModeIsActive() && timer.seconds() < t) {
            if(isTeleOp){
                if(myOpMode.gamepad1.right_stick_button){
                    break;
                }
            }
            robot.armControl(0, 1);
        }
    }

    public void teleopInitialize(RobotControl r, GoBildaPinpointDriver o, EnhancedNavigation n){
        robot = r;
        odo = o;
        navigation = n;
        isTeleOp = true;
        power = 1;
        weird = 1;
    }
}