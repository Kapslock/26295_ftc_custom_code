package org.firstinspires.ftc.teamcode.testSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;

@TeleOp
public class TestMotorOrientation extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
    }

    @Override
    public void loop() {
        if (gamepad1.square) {frontLeftMotor.setPower(1);} else {frontLeftMotor.setPower(0);}
        if (gamepad1.triangle) {frontRightMotor.setPower(1);} else {frontRightMotor.setPower(0);}
        if (gamepad1.circle) {backRightMotor.setPower(1);} else {backRightMotor.setPower(0);}
        if (gamepad1.cross) {backLeftMotor.setPower(1);} else {backLeftMotor.setPower(0);}
    }
}
