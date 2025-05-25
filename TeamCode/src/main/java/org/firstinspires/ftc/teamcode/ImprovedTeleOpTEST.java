package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name = "Dynamic Thread Field-Centric Omni OpMode with Slide PID", group = "Linear OpMode")
public class ImprovedTeleOpTEST extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // Drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Slide motors
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    // IMU
    private BNO055IMU imu;
    private double headingOffset = 0;
    private boolean isFieldCentric = true;

    // Joystick smoothing variables (mutable)
    private double smoothedAxial = 0.0;
    private double smoothedLateral = 0.0;
    private double smoothedYaw = 0.0;

    private final ExecutorService executor = Executors.newCachedThreadPool();
    private final AtomicBoolean isBraking = new AtomicBoolean(false);

    private final double[] lastMotorPowers = new double[]{0, 0, 0, 0};

    // Drive constants
    private static final double DEADBAND = 0.05;
    private static final double SMOOTHING_FACTOR = 0.15;
    private static final int BRAKE_DURATION_MS = 500;
    private static final double BRAKE_POWER = -0.8;  // Base brake power

    // Slide PID state
    private double integralSum = 0;
    private double lastError = 0;

    private int targetPosition = 0;

    // Debounce flags
    private boolean previousB = false;
    private boolean previousY = false;
    private boolean previousX = false;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    // Brake polarity toggle flag
    private boolean isBrakePowerNegative = true;

    private volatile boolean telemetryThreadActive = true;

    @Override
    public void runOpMode() {
        initializeHardware();
        initializeIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Start telemetry thread to update continuously
        Thread telemetryThread = new Thread(() -> {
            while (!isStopRequested() && telemetryThreadActive) {
                updateTelemetry();
                try {
                    Thread.sleep(100); // Update every 100ms (10 Hz)
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        telemetryThread.start();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            handleBrakePolarityToggle();
            handleDriveControls();
            handleSlideControls();
            // Telemetry handled in separate thread
        }

        telemetryThreadActive = false;
        try {
            telemetryThread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        stopAllMotors();
        executor.shutdownNow();
    }

    private void initializeHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Keep RUN_WITHOUT_ENCODER for manual PID control in code
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initializeIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        Parameters parameters = new Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu.initialize(parameters);

        telemetry.addData("IMU", "Initializing...");
        telemetry.update();

        ElapsedTime calibrationTimer = new ElapsedTime();
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            telemetry.addData("IMU Status", imu.getCalibrationStatus().toString());
            telemetry.addData("IMU", "Calibrating... %.1f sec", calibrationTimer.seconds());
            telemetry.update();
            sleep(100);

            if (calibrationTimer.seconds() > 10) {
                telemetry.addData("IMU", "Calibration timeout. Proceeding anyway.");
                telemetry.update();
                break;
            }
        }

        telemetry.addData("IMU", "Calibrated or Skipped");
        telemetry.addData("Calibration Status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    private void handleDriveControls() {
        boolean currentB = gamepad1.b;
        if (currentB && !previousB) {
            headingOffset = getIMUOrientation().firstAngle;
            telemetry.addData("IMU", "Heading reset");
            telemetry.update();
        }
        previousB = currentB;

        boolean currentY = gamepad1.y;
        if (currentY && !previousY) {
            isFieldCentric = !isFieldCentric;
        }
        previousY = currentY;

        if (gamepad1.left_trigger > 0.1 && isBraking.compareAndSet(false, true)) {
            executor.submit(() -> {
                applyBrake();
                isBraking.set(false);
            });
        }

        // Apply cubic joystick scaling for finer control
        double rawAxial = applyDeadzone(-gamepad1.left_stick_y);
        double rawLateral = applyDeadzone(gamepad1.left_stick_x);
        double rawYaw = applyDeadzone(gamepad1.right_stick_x);

        rawAxial = cubicScaling(rawAxial);
        rawLateral = cubicScaling(rawLateral);
        rawYaw = cubicScaling(rawYaw);

        updateJoystickInputs(rawAxial, rawLateral, rawYaw);

        double speedFactor = (gamepad1.right_trigger > 0.1) ? 0.4 : 1.0;

        double heading = getHeadingRadians();

        double rotatedAxial, rotatedLateral;
        if (isFieldCentric) {
            rotatedAxial = smoothedAxial * Math.cos(heading) + smoothedLateral * Math.sin(heading);
            rotatedLateral = -smoothedAxial * Math.sin(heading) + smoothedLateral * Math.cos(heading);
        } else {
            rotatedAxial = smoothedAxial;
            rotatedLateral = smoothedLateral;
        }

        double[] powers = calculateMotorPowers(rotatedAxial, rotatedLateral, smoothedYaw, speedFactor);

        if (!arePowersEqual(powers, lastMotorPowers)) {
            setMotorPowers(powers);
            System.arraycopy(powers, 0, lastMotorPowers, 0, powers.length);
        }
    }

    private void updateJoystickInputs(double rawAxial, double rawLateral, double rawYaw) {
        smoothedAxial += (rawAxial - smoothedAxial) * SMOOTHING_FACTOR;
        smoothedLateral += (rawLateral - smoothedLateral) * SMOOTHING_FACTOR;
        smoothedYaw += (rawYaw - smoothedYaw) * SMOOTHING_FACTOR;
    }

    private void handleSlideControls() {
        // Preset positions
        final int maxPosition = 1000;
        final int minPosition = 0;

        if (gamepad2.a) {
            targetPosition = minPosition;
        } else if (gamepad2.y) {
            targetPosition = (minPosition + maxPosition) / 2;
        } else if (gamepad2.b) {
            targetPosition = maxPosition;
        } else if (gamepad2.x) {
            targetPosition = getSlidePosition();
        }

        // Manual fine control with left stick
        double stickY = -gamepad2.left_stick_y;
        double slideDeadzone = 0.05;
        int fineAdjustAmount = 10;
        if (Math.abs(stickY) >= slideDeadzone) {
            targetPosition += (int) (stickY * fineAdjustAmount);
        }

        // D-pad fine adjustment with debounce
        if (gamepad2.dpad_up && !dpadUpPrev) {
            targetPosition += fineAdjustAmount;
        }
        if (gamepad2.dpad_down && !dpadDownPrev) {
            targetPosition -= fineAdjustAmount;
        }
        dpadUpPrev = gamepad2.dpad_up;
        dpadDownPrev = gamepad2.dpad_down;

        // Clamp target position
        targetPosition = clamp(targetPosition);

        int currentPosition = getSlidePosition();
        int error = targetPosition - currentPosition;

        // Integral windup protection with smaller clamp
        integralSum = clamp(integralSum + error, -500, 500);

        double derivative = error - lastError;

        final double kP = 0.004;
        final double kI = 0.0001;
        final double kD = 0.001;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);
        output = clamp(output, -1, 1);

        lastError = error;

        leftSlide.setPower(output);
        rightSlide.setPower(output);
    }

    private void applyBrake() {
        // Determine brake power polarity based on toggle
        double brakePower = isBrakePowerNegative ? BRAKE_POWER : -BRAKE_POWER;

        leftFrontDrive.setPower(brakePower);
        leftBackDrive.setPower(brakePower);
        rightFrontDrive.setPower(brakePower);
        rightBackDrive.setPower(brakePower);

        sleep(BRAKE_DURATION_MS);

        // Stop motors after brake
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void toggleBrakePolarity() {
        isBrakePowerNegative = !isBrakePowerNegative;
    }

    private void handleBrakePolarityToggle() {
        boolean currentX = gamepad1.x;
        if (currentX && !previousX) {
            toggleBrakePolarity();
            telemetry.addData("Brake Polarity", isBrakePowerNegative ? "Negative" : "Positive");
            telemetry.update();
        }
        previousX = currentX;
    }

    private void stopAllMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }

    private void setMotorPowers(double[] powers) {
        leftFrontDrive.setPower(powers[0]);
        leftBackDrive.setPower(powers[1]);
        rightFrontDrive.setPower(powers[2]);
        rightBackDrive.setPower(powers[3]);
    }

    private double applyDeadzone(double input) {
        return (Math.abs(input) < DEADBAND) ? 0 : input;
    }

    private double cubicScaling(double input) {
        return input * input * input; // cube input for fine control near zero
    }

    private boolean arePowersEqual(double[] a, double[] b) {
        final double EPSILON = 0.001;
        for (int i = 0; i < a.length; i++) {
            if (Math.abs(a[i] - b[i]) > EPSILON) {
                return false;
            }
        }
        return true;
    }

    private double[] calculateMotorPowers(double axial, double lateral, double yaw, double speedFactor) {
        double lfPower = axial + lateral + yaw;
        double lbPower = axial - lateral + yaw;
        double rfPower = axial - lateral - yaw;
        double rbPower = axial + lateral - yaw;

        // Normalize if any power exceeds 1
        double maxPower = Math.max(Math.abs(lfPower), Math.max(Math.abs(lbPower),
                Math.max(Math.abs(rfPower), Math.abs(rbPower))));

        if (maxPower > 1.0) {
            lfPower /= maxPower;
            lbPower /= maxPower;
            rfPower /= maxPower;
            rbPower /= maxPower;
        }

        // Apply speed factor
        lfPower *= speedFactor;
        lbPower *= speedFactor;
        rfPower *= speedFactor;
        rbPower *= speedFactor;

        return new double[]{lfPower, lbPower, rfPower, rbPower};
    }

    private Orientation getIMUOrientation() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    private double getHeadingRadians() {
        // Negative sign is intentional to align IMU with robot coordinate frame
        return -getIMUOrientation().firstAngle + headingOffset;
    }

    private int getSlidePosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2;
    }

    private int clamp(int val) {
        return Math.max(0, Math.min(1000, val));
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private void updateTelemetry() {
        int currentSlidePos = getSlidePosition();
        telemetry.addData("Runtime", "%.2f s", runtime.seconds());
        telemetry.addData("Drive Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Heading (deg)", Math.toDegrees(getHeadingRadians()));
        telemetry.addData("Target Slide Pos", targetPosition);
        telemetry.addData("Current Slide Pos", currentSlidePos);
        telemetry.addData("Slide Error", targetPosition - currentSlidePos);
        telemetry.addData("Brake Polarity", isBrakePowerNegative ? "Negative" : "Positive");
        telemetry.update();
    }
}
