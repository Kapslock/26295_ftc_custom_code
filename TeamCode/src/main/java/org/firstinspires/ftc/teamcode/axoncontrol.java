package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Axon CRServo Control", group = "TeleOp")
public class axoncontrol extends LinearOpMode {

    private CRServo axon1;
    private CRServo axon2;

    @Override
    public void runOpMode() {
        axon1 = hardwareMap.get(CRServo.class, "axon1");
        axon2 = hardwareMap.get(CRServo.class, "axon2");

        axon1.setPower(0.0);
        axon2.setPower(0.0);

        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        waitForStart();

        boolean lastDpadRight = false;
        boolean isRunningSequence = false;

        while (opModeIsActive()) {
            boolean currentDpadRight = gamepad1.dpad_right;

            // Edge detection for button press
            if (currentDpadRight && !lastDpadRight && !isRunningSequence) {
                isRunningSequence = true;
                runSequence();
                isRunningSequence = false;
            }

            lastDpadRight = currentDpadRight;

            telemetry.addData("Axon1 Power", axon1.getPower());
            telemetry.addData("Axon2 Power", axon2.getPower());
            telemetry.update();
        }
    }

    private void runSequence() {
        telemetry.addLine("Sequence started");
        telemetry.update();

        // Step 1: Rotate both left (backward) 90 degrees
        axon1.setPower(-1.0);
        axon2.setPower(1.0);
        sleep(500);

        // Step 2: Wait
        axon1.setPower(0.0);
        axon2.setPower(0.0);
        sleep(500);

        // Step 3: Rotate both right (forward) 90 degrees to return to zero
        axon1.setPower(1.0);
        axon2.setPower(-1.0);
        sleep(500);

        // Step 4: Wait
        axon1.setPower(0.0);
        axon2.setPower(0.0);
        sleep(500);

        // Step 5: Reverse axon2, rotate both forward 90 degrees
        axon1.setPower(1.0);
        axon2.setPower(1.0); // reversed "forward" now becomes forward
        sleep(500);

        // Step 6: Rotate both backward 90 degrees
        axon1.setPower(-1.0);
        axon2.setPower(-1.0);
        sleep(500);

        // Stop
        axon1.setPower(0.0);
        axon2.setPower(0.0);

        telemetry.addLine("Sequence complete");
        telemetry.update();
    }
}
