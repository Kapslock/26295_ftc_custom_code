package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.teleOp.GoBildaPinpointDriver;

import java.util.Locale;

public class MecanumDrive {

    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    IMU imu;
    GoBildaPinpointDriver odo;

    public void init(HardwareMap hwMap, Telemetry telemetry) {

        //Hardware Mapping
        frontLeftMotor = hwMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hwMap.get(DcMotor.class, "back_right_motor");

        odo = hwMap.get(GoBildaPinpointDriver.class,"odo");

        imu = hwMap.get(IMU.class, "imu");

        //Drive Motor Spin Directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        //Motors Run Using or Without Encoder
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Brake at 0 Power
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Meet 0 Bot Offsets: -0.4, 3.4, INCH
        odo.setOffsets(-0.4, 3.4, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
        DO THIS: MOUNT PINPOINT COMPUTER WITH LOGO FACING UP!!!
        https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf
        1. Mount Pinpoint sticker-side up on the chassis.
        6. Check that the Device Status is Ready or that the LED on the Pinpoint is green.
            If it is Purple, Blue, or Orange, check your pods and make sure they’re correctly connected.
        7. Move the robot forward without rotating it. Make sure that the estimated X position increases.
            If X decreases, reverse the X Encoder direction by setting the X Encoder Direction to reversed.
            If Y moves more than X, make sure that the pod plugged into the X port is tracking forward.
            If neither value moves, make sure that your X pod is working.
        8. Move the robot left without rotating it. Make sure that the Y position increases.
            If Y decreases, reverse the Y Encoder Direction.
            If neither value moves, make sure that your Y pod is working.
        9. Measure your Pod Offsets as described on page 2/3, and write those offsets to the Pinpoint.
        10. Rotate your robot around the tracking point without sliding the robot forward or sideways. The
            X and Y positions should stay fairly low. If they fluctuate by more than ~100mm or 4”, double
            check your Pod Offsets. The most common issue here is that one of the offsets is positive when
            it needs to be negative.
        11. Rotate the robot a full turn counterclockwise. The heading should read very close to exactly one
            rotation positive.
        12. Using a tape measure, move your robot along the field and measure how far it has moved, and
            compare it to how far the Pinpoint reports it moving. If these are significantly different, double
            check your ticks-per-mm configuration.
        13. Reset the estimated position (with the button or with your code) and drive the robot around a
            little bit before returning to the starting point. The values should be close to zero. If everything is
            working, expect to see your robot return to within about 10mm (~0.5”) of the starting position.
            More is a good indicator that something is wrong.
        */

        //Meet 0 Bot Directions: FORWARD, FORWARD
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //Calibrate ODO
        odo.resetPosAndIMU();
        odo.recalibrateIMU();


        //Initialize IMU
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset (Inches)", odo.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Y offset (Inches)", odo.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Odo Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Odo Heading Scalar", odo.getYawScalar());

        telemetry.update();

    }

    public void drive(double forward, double strafe, double rotate, double slow) {

        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe - rotate;
        double frontRightPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(slow * maxSpeed * (frontLeftPower / maxPower));
        frontRightMotor.setPower(slow * maxSpeed * (backLeftPower / maxPower));
        backLeftMotor.setPower(slow * maxSpeed * (frontRightPower / maxPower));
        backRightMotor.setPower(slow * maxSpeed * (backRightPower / maxPower));

    }

    public void driveFieldOriented(double forward, double strafe, double rotate, double slow, Telemetry telemetry) {

        //Converts X, Y coordinates to polar coordinates
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

        /* Use this code to use the IMU instead of the odo:
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)); */

        //Recalculates heading based on current position
        double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);
        theta = AngleUnit.normalizeRadians(theta - heading);

        /*
        Use this code to use the IMU instead of the odo:
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        */

        //Converts values back to X, Y coordinates from polar
        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        //General Odo Tele
        telemetry.addData("New Forward", newForward);
        telemetry.addData("New Strafe", newStrafe);
        telemetry.addData("Theta (Radians)", theta);
        telemetry.addData("Odo Status", odo.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints the current refresh rate of the Pinpoint
        telemetry.addLine();

        //Heading Tele
        telemetry.addData("Heading (deg)", odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addLine();

        //X, Y Pos from Odo Tele
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addLine();

        //Velocity from Odo Tele
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}",
                odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM),
                odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);

        telemetry.update();

        //Uses modified Field oriented factors in regular drive class
        this.drive(newForward, newStrafe, rotate, slow);

    }

    public void OdoReset(Telemetry tele) {

        //Resets Heading and Position -STAY STILL FOR AT LEAST 0.25 SECONDS WHILE DOING SO FOR ACCURACY-
        odo.resetPosAndIMU();
        odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        tele.update();

    }

}