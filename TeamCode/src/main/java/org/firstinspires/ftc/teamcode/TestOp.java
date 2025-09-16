package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TestOp", group="Linear OpMode")
public class TestOp extends LinearOpMode {

    private DcMotor skibidi = null;
    private DcMotor skibidii = null;
    private DcMotor skibidiii = null;
    private DcMotor skibidiiii = null;

    @Override
    public void runOpMode() {
        skibidi = hardwareMap.get(DcMotor.class, "left_front_drive");
        skibidii = hardwareMap.get(DcMotor.class, "left_back_drive");
        skibidiii = hardwareMap.get(DcMotor.class, "right_front_drive");
        skibidiiii = hardwareMap.get(DcMotor.class, "right_back_drive");


        skibidi.setPower(1);
        skibidii.setPower(1);
        skibidiii.setPower(1);
        skibidiiii.setPower(1);

        while (opModeIsActive()) {
        }

        skibidi.setPower(0);
        skibidii.setPower(0);
        skibidiii.setPower(0);
        skibidiiii.setPower(0);
    }
}
