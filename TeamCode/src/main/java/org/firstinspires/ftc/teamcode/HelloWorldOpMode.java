package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Locale;

@TeleOp

// TODO mettre une enum Mode(OFF AUTODRIVE TELEOP)

    private DcMotor backLeftDrive, backRightDrive, frontLeftDrive, frontRightDrive;
    private IMU imu;

    private void initializeHardware() {
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("initializeHardware", "done");
        telemetry.update();
    }

    private void processDriveInputs(float turn, float forward, float strafe) {
        // TODO : besoin du while(isActive ?)

        // Combine inputs to create drive and turn (or both!)
        float frontLeftPower = forward + turn + strafe;
        float frontRightPower = forward + turn - strafe;
        float backLeftPower = forward - turn + strafe;
        float backRightPower = forward + turn + strafe;

        // Normalize wheel powers to be less than 1.0 ?
        float max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
        telemetry.addData("processDriveInputs", String.format(Locale.ENGLISH, "turn: %2f forward: %2f strafe: %2f", turn, forward, strafe));
        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("frontRightPower: ", frontRightPower);
        telemetry.addData("backLeftPower: ", backLeftPower);
        telemetry.addData("backRightPower: ", backRightPower);
        telemetry.update();
    }

    private void stopWheels() {
        processDriveInputs(0, 0, 0);
    }

    private void slideRightFor(int millis) {
        telemetry.addData("autodrive", "slide right");
        telemetry.update();

        processDriveInputs(1, -1, 1);
        sleep(millis);
        stopWheels();
    }

    private void slideLeftFor(int millis) {
        telemetry.addData("autodrive", "slide left");
        telemetry.update();

        processDriveInputs(-1, 1, -1);
        sleep(millis);
        stopWheels();
    }

    private void turnLeftFor(int millis) {
        telemetry.addData("autodrive", "left");
        telemetry.update();

        processDriveInputs(1, 0, -1);
        sleep(millis);
        stopWheels();
    }

    private void turnRightFor(int millis) {
        telemetry.addData("autodrive", "right");
        telemetry.update();

        processDriveInputs(-1, 0, 1);
        sleep(millis);
        stopWheels();
    }

    private void moveForwardFor(int millis) {
        telemetry.addData("autodrive", "forward");
        telemetry.update();

        processDriveInputs(0, 1, 0);
        sleep(millis);
        stopWheels();
    }

    private void moveBackwardFor(int millis) {
        telemetry.addData("autodrive", "backward");
        telemetry.update();

        processDriveInputs(0, -1, 0);
        sleep(millis);
        stopWheels();
    }

    // pivote autour de la roue avant gauche vers la droite
    private void turnUpRightFor(int millis) {
        telemetry.addData("autodrive", "turn right");
        telemetry.update();

        processDriveInputs(-1, 0, 0);
        sleep(millis);
        stopWheels();
    }

    // pivote autour de la roue avant gauche vers la gauche
    private void turnUpLeftFor(int millis) {
        telemetry.addData("autodrive", "turn left");
        telemetry.update();

        processDriveInputs(1, 0, 0);
        sleep(millis);
        stopWheels();
    }

    // pivote autour de la roue arriere droite vers la droite
    private void turnDownRightFor(int millis) {
        telemetry.addData("autodrive", "turn right");
        telemetry.update();

        processDriveInputs(0, 0, 1);
        sleep(millis);
        stopWheels();
    }

    // pivote autour de la roue arriere droite vers la gauche
    private void turnDownLeftFor(int millis) {
        telemetry.addData("autodrive", "turn left");
        telemetry.update();

        processDriveInputs(0, 0, -1);
        sleep(millis);
        stopWheels();
    }

    private void drawCircle() {
        telemetry.addData("autodrive", "draw circle");
        telemetry.update();

        processDriveInputs(1, 1, 0);
        sleep(5000);
    }


    public void autoDriveMode() {
        // go to the center
        slideRightFor(300);

        // forward
        moveForwardFor(1500);

        // pivot
        turnUpLeftFor(3500);
        turnUpRightFor(3500);

        turnDownLeftFor(3500);
        turnDownRightFor(3500);

        turnLeftFor(3500);
        turnRightFor(3500);

        slideLeftFor(1000);
        slideRightFor(2000);
        slideLeftFor(1000);
    }

    private void gamepadDriveMode() {
        while (opModeIsActive()) {
            float turn = gamepad1.right_stick_x;
            float forward = gamepad1.left_stick_y;
            float strafe = gamepad1.left_stick_x;
            processDriveInputs(turn, forward, strafe);
        }
    }

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();

        autoDriveMode();
    }
}
