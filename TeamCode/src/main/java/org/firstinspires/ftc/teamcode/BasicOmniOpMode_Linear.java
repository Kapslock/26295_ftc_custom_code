package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp(name = "Enhanced: Omni Linear OpMode", group = "Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private final ExecutorService executor = Executors.newFixedThreadPool(4);

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double axial = applyDeadzone(-gamepad1.left_stick_y);
            double lateral = applyDeadzone(gamepad1.left_stick_x);
            double yaw = applyDeadzone(gamepad1.right_stick_x);

            double speedFactor = (gamepad1.left_trigger > 0.1) ? 0.4 : 1.0;

            double[] powers = calculateMotorPowers(axial, lateral, yaw, speedFactor);

            setMotorPowersAsync(powers);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", powers[0], powers[1]);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", powers[2], powers[3]);
            telemetry.update();
        }

        executor.shutdownNow();
    }

    private void initializeHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    private double applyDeadzone(double value) {
        return (Math.abs(value) > 0.05) ? value : 0.0;
    }

    private double[] calculateMotorPowers(double axial, double lateral, double yaw, double speedFactor) {
        double leftFrontPower = (axial + lateral + yaw);
        double rightFrontPower = (axial - lateral - yaw);
        double leftBackPower = (axial - lateral + yaw);
        double rightBackPower = (axial + lateral - yaw);

        // Normalize powers
        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Apply speed factor
        return new double[] {
                leftFrontPower * speedFactor,
                rightFrontPower * speedFactor,
                leftBackPower * speedFactor,
                rightBackPower * speedFactor
        };
    }

    private void setMotorPowersAsync(double[] powers) {
        executor.submit(() -> leftFrontDrive.setPower(powers[0]));
        executor.submit(() -> rightFrontDrive.setPower(powers[1]));
        executor.submit(() -> leftBackDrive.setPower(powers[2]));
        executor.submit(() -> rightBackDrive.setPower(powers[3]));
    }
}
