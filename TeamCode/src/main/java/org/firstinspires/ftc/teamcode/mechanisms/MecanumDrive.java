package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    private DcMotor backRightMotor, frontRightMotor, backLeftMotor, frontLeftMotor;
    private IMU imu;

    public void init(HardwareMap hwMap) {
        backRightMotor = hwMap.get(DcMotor.class, "back_right_motor");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");
        frontLeftMotor = hwMap.get(DcMotor.class, "front_left_motor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(revOrientation));
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe - rotate;
        double backLeftPower = forward - strafe - rotate;
        double frontRightPower = forward - strafe + rotate;
        double backRightPower = forward + strafe + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxSpeed = Math.max(maxPower, Math.abs(frontLeftPower));
        maxSpeed = Math.max(maxPower, Math.abs(backLeftPower));
        maxSpeed = Math.max(maxPower, Math.abs(frontRightPower));
        maxSpeed = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftMotor.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightMotor.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightMotor.setPower(maxSpeed * (backRightPower / maxPower));
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double r = Math.hypot(strafe, forward);
//
//        theta = AngleUnit.normalizeRadians(theta -t
//                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double newForward = strafe * Math.sin(theta) + forward * Math.cos(theta);
        double newStrafe = strafe * Math.cos(theta) - forward * Math.sin(theta);

        this.drive(newForward, newStrafe, rotate);
    }
}
