package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class Motor_Run extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain driveTrain = new DriveTrain(this);

        final DcMotor OutakeMotor = hardwareMap.dcMotor.get("OuttakeMotor");
        final IMU imu;



        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            OutakeMotor.setPower(gamepad1.right_trigger);


        }
    }
}
