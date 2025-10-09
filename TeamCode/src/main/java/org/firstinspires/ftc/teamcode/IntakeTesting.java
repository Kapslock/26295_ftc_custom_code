package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test intake motor", group = "examples")
public class IntakeTesting extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode() {
        // Map the motor from the config (make sure your config name matches!)
        motor = hardwareMap.get(DcMotor.class, "motor");

        // Optional: set direction if needed
        // motor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for Play button
        waitForStart();

        while (opModeIsActive()) {
            // Read controller input (right trigger: 0.0 to 1.0)
            double power = gamepad1.right_trigger;

            // Set motor power
            motor.setPower(power);

            // Telemetry for debugging
            telemetry.addData("Trigger", power);
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.update();
        }
    }
}
