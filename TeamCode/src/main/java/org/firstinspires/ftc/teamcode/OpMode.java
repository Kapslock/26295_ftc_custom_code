package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class OpMode extends LinearOpMode {

    private DcMotor FRW, FLW, BLW, BRW;

    @Override
    public void runOpMode() throws InterruptedException {
//tells the code which motors it controls
        FRW = hardwareMap.get(DcMotor.class, "FRW");
        FLW = hardwareMap.get(DcMotor.class, "FLW");
        BLW = hardwareMap.get(DcMotor.class, "BLW");
        BRW = hardwareMap.get(DcMotor.class, "BRW");
//once play is pressed then it will start
        waitForStart();
//loop runs over and over while OpMode is active.
        //gamepad checks and telemetry
        while (opModeIsActive()) {
            if (gamepad1.a) {
                //move forwards
                FLW.setPower(0.5);
                FRW.setPower(0.5);
                BLW.setPower(0.5);
                BRW.setPower(0.5);


            } else if (gamepad1.b) {
                //move back
                FLW.setPower(-0.5);
                FRW.setPower(-0.5);
                BLW.setPower(-0.5);
                BRW.setPower(-0.5);

            } else {
                //stop
                FLW.setPower(0);
                FRW.setPower(0);
                BLW.setPower(0);
                BRW.setPower(0);

            }
//see button presses and joystick values
            telemetry.addData("Gamepad1 A Pressed", gamepad1.a);
            telemetry.addData("Gamepad1 B Pressed", gamepad1.b);
            telemetry.addData("Left Stick", gamepad1.left_stick_y);//moves left motor
            telemetry.addData("Right Stick", gamepad1.right_stick_y);//moves right motor
            telemetry.update();

        }
    }

}




