package org.firstinspires.ftc.teamcode.BBcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = " Test Launcher" )
public class Test_Launcher extends LinearOpMode {
    DcMotorEx launcher;
    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses START)
        waitForStart();



        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotor.Direction.FORWARD);

        double requestedPower = 0;
        double actualPower = 0;
        boolean isActive = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) {
                requestedPower += .01;
            }
            if (gamepad1.dpadDownWasPressed()) {
                requestedPower -= .01;
            }
            if (gamepad1.aWasPressed()) {
                isActive = false;
            }
            if (gamepad1.yWasPressed()) {
                isActive = true;
            }
            //launcherPower = Math.max(-1, Math.min(1, launcherPower));
            if (isActive) {
                actualPower = Math.max(-1, Math.min(1, requestedPower));
            } else {
                actualPower = 0;
            }
            launcher.setPower(actualPower);
            telemetry.addData("Requested Power", requestedPower);
            telemetry.addData("Launcher Velocity", launcher.getVelocity());
            telemetry.addData("Actual Power", launcher.getPower());

            telemetry.update();
        }
    }
}
