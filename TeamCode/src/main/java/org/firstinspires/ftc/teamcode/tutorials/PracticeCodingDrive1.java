package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class PracticeCodingDrive1 extends OpMode {

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
        boolean aPressed;
        if (gamepad1.a) {
            aPressed = true;
        } else {
            aPressed = false;
        }
        telemetry.addData("A button prees?", aPressed);
    }
}
