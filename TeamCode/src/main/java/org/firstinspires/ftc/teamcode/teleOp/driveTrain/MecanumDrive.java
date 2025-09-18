package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleOp.GoBildaPinpointDriver;

public class MecanumDrive {

    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    GoBildaPinpointDriver odo;
    private IMU imu;

    public void init(HardwareMap hwMap, Telemetry telemetry) {

        frontLeftMotor = hwMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hwMap.get(DcMotor.class, "back_right_motor");

        odo = hwMap.get(GoBildaPinpointDriver.class,"odo");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(RevOrientation));

        //check these
        odo.setOffsets(1, 7.8, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

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

    }

    public void driveFieldOriented(double forward, double strafe, double rotate, double slow, Telemetry telemetry) {

        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

        double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);
        theta = AngleUnit.normalizeRadians(theta - heading);

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        telemetry.addData("New Forward", newForward);
        telemetry.addData("New Strafe", newStrafe);
        telemetry.addData("Theta", theta);
        telemetry.addData("Imu Angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Heading (rad)", heading);
        telemetry.addData("Heading (deg)", odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("Odo Status", odo.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

        telemetry.update();

        this.drive(newForward, newStrafe, rotate, slow);
    }

}
