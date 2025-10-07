package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.Locale;

public class MecanumDrive {

    private static final Logger log = LoggerFactory.getLogger(MecanumDrive.class);
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    GoBildaPinpointDriver odo;
    public IMU imu;
    private PIDcontroller headingPID;  // PID controller for heading
    private PIDcontroller drivePID;    // Optional PID for forward/backward distance

    public void init(HardwareMap hwMap, Telemetry telemetry) {

        frontLeftMotor = hwMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hwMap.get(DcMotor.class, "back_right_motor");

        odo = hwMap.get(GoBildaPinpointDriver.class,"odo");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //New Bot Offsets
        //odo.setOffsets(-0.5, 0, DistanceUnit.INCH);
        odo.setOffsets(-0.4, 3.6, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        double kp = 0.5;
        double ki = 0.0;
        double kd = 0.0;

        headingPID = new PIDcontroller(kp, ki, kd); // tune these values
        headingPID.setTarget(0); // default target heading = 0 degrees
        String data = String.format(Locale.US, "{KP: %.3f, KI: %.3f, KD: %.3f}", kp, kd, ki);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset (Inches)", odo.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Y offset (Inches)", odo.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Odo Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Odo Heading Scalar", odo.getYawScalar());
        telemetry.addData("PID Settings", data);
        telemetry.update();

    }

    public void drive(double forward, double strafe, double rotate, double slow, Telemetry telemetry) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 0.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeftMotor.setPower(slow * maxSpeed * (frontLeftPower));
        frontRightMotor.setPower(slow * maxSpeed * (frontRightPower));
        backLeftMotor.setPower(slow * maxSpeed * (backLeftPower));
        backRightMotor.setPower(slow * maxSpeed * (backRightPower));

        telemetry.addData("Front Left Motor Power", maxSpeed * (frontLeftPower));
        telemetry.addData("Front Right Motor Power", maxSpeed * (frontRightPower));
        telemetry.addData("Back Left Motor Power", maxSpeed * (backLeftPower));
        telemetry.addData("Back Right Motor Power", maxSpeed * (backRightPower));
    }

    public void driveFieldOriented(double forward, double strafe, double rotate, double slow, Telemetry telemetry) {

        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        odo.update();

        double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);
        theta = AngleUnit.normalizeRadians(theta - heading);

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));

        telemetry.addData("New Forward", newForward);
        telemetry.addData("New Strafe", newStrafe);
        telemetry.addData("Theta", theta);
        telemetry.addData("Odo Status", odo.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
        telemetry.addLine();
        telemetry.addData("Position", data);
        telemetry.addLine();
        telemetry.addData("Heading (deg)", odo.getPosition().getHeading(AngleUnit.DEGREES));

        this.drive(newForward, newStrafe, rotate, slow, telemetry);

        telemetry.update();
    }

    public void turnToHeading(double targetHeading, double slow, Telemetry telemetry, double kp, double ki, double kd) {

        headingPID.setKP(kp);
        headingPID.setKI(ki);
        headingPID.setKD(kd);
        headingPID.setTarget(targetHeading);

        odo.update();
        double currentHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
        double time = System.nanoTime() / 1e9; // seconds

        double correction = headingPID.calculateOutput(currentHeading, time);

        // Apply correction as rotation power
        double rotate = correction;

        // Keep forward/strafe 0, just rotate
        this.drive(0, 0, rotate, slow, telemetry);

        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addData("Correction (PID)", correction);
        telemetry.update();
    }

    public void OdoReset(Telemetry tele) {

        odo.resetPosAndIMU();
        odo.update();
        tele.update();

    }

}
