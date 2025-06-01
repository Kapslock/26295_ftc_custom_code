package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name = "ImprovedTeleOpTEST", group = "Linear OpMode")
public class ImprovedTeleOpTEST extends LinearOpMode {

    // Runtime timer for telemetry and operation timing
    private final ElapsedTime runtime = new ElapsedTime();

    // Drive motors
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    // Slide motors for the linear slide mechanism
    private DcMotor leftSlide, rightSlide;
    // IMU sensor for heading and orientation
    private BNO055IMU imu;

    // Heading offset for field-centric control reset
    private double headingOffset = 0;
    // Flag to toggle field-centric vs robot-centric drive
    private boolean isFieldCentric = true;

    // Smoothed joystick inputs for smoother drive response
    private double smoothedAxial = 0, smoothedLateral = 0, smoothedYaw = 0;
    // Last applied motor powers to reduce redundant commands
    private final double[] lastMotorPowers = new double[]{0, 0, 0, 0};

    // Executor for asynchronous braking thread
    private final ExecutorService executor = Executors.newCachedThreadPool();
    // Atomic flag to indicate if braking is currently running
    private final AtomicBoolean isBraking = new AtomicBoolean(false);
    // Control flag for telemetry thread lifecycle
    private volatile boolean telemetryThreadActive = true;

    // PID control variables for slide position control
    private int targetPosition = 0;   // Desired slide encoder position
    private double integralSum = 0;   // Integral accumulator for PID
    private int lastError = 0;        // Last position error for derivative term

    // Variables for dynamic slide range control using X button
    private int xPressCount = 0;      // Counts X button presses for min/max reset
    private int slideMin = 150;       // Minimum allowed slide position
    private int slideMax = 1300;      // Maximum allowed slide position

    // Button state trackers for edge detection (debounce)
    private boolean previousB = false;
    private boolean previousY = false;
    private boolean previousX = false;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    // Brake power polarity toggle state
    private boolean isBrakePowerNegative = true;

    // Variables for detecting long press on X button to reset slide range
    private long xButtonPressedTime = 0;
    private final long xButtonHoldThreshold = 1000; // milliseconds

    @Override
    public void runOpMode() {
        // Initialize motors and sensors
        initializeHardware();
        initializeIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Start a separate thread to update telemetry periodically
        Thread telemetryThread = new Thread(() -> {
            while (!isStopRequested() && telemetryThreadActive) {
                updateTelemetry();
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        telemetryThread.start();

        // Wait for start button press
        waitForStart();
        runtime.reset();

        // Main control loop runs while op mode is active
        while (opModeIsActive()) {
            handleBrakePolarityToggle();
            handleDriveControls();
            handleSlideControls();
        }

        // Shutdown telemetry thread and motors on stop
        telemetryThreadActive = false;
        try {
            telemetryThread.join();
        } catch (InterruptedException ignored) {
        }
        stopAllMotors();
        executor.shutdownNow();
    }

    /**
     * Initializes drive and slide motors with directions and encoder modes.
     */
    private void initializeHardware() {
        // Map drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // Reverse left side motors for proper orientation
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Map slide motors and set their directions
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders and run without encoder for slide motors (PID control manual)
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Initializes and calibrates the IMU sensor.
     */
    private void initializeIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        Parameters params = new Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(params);

        // Wait for gyro calibration up to 10 seconds
        ElapsedTime timer = new ElapsedTime();
        while (!isStopRequested() && !imu.isGyroCalibrated() && timer.seconds() < 10) {
            telemetry.addData("IMU", "Calibrating...");
            telemetry.update();
            sleep(100);
        }
        telemetry.addData("IMU", "Ready");
        telemetry.update();
    }

    /**
     * Handles driving control: field-centric toggle, heading reset,
     * joystick input smoothing, speed scaling, and motor power setting.
     */
    private void handleDriveControls() {
        // Reset heading offset on B button press (rising edge)
        if (gamepad1.b && !previousB) {
            headingOffset = getIMUOrientation().firstAngle;
        }
        previousB = gamepad1.b;

        // Toggle between field-centric and robot-centric drive on Y button press
        if (gamepad1.y && !previousY) {
            isFieldCentric = !isFieldCentric;
        }
        previousY = gamepad1.y;

        // Initiate brake asynchronously on left trigger press if not already braking
        if (gamepad1.left_trigger > 0.1 && isBraking.compareAndSet(false, true)) {
            executor.submit(() -> {
                applyBrake();
                isBraking.set(false);
            });
        }

        // Read and smooth joystick inputs with cubic scaling for finer control
        double rawAxial = cubicScaling(applyDeadzone(-gamepad1.left_stick_y));
        double rawLateral = cubicScaling(applyDeadzone(gamepad1.left_stick_x));
        double rawYaw = cubicScaling(applyDeadzone(gamepad1.right_stick_x));

        smoothedAxial += (rawAxial - smoothedAxial) * 0.15;
        smoothedLateral += (rawLateral - smoothedLateral) * 0.15;
        smoothedYaw += (rawYaw - smoothedYaw) * 0.15;

        // Get heading in radians from IMU, apply offset and invert sign
        double heading = getHeadingRadians();

        // Speed scaling with right trigger (slower if pressed)
        double speed = (gamepad1.right_trigger > 0.1) ? 0.4 : 1.0;

        // Apply field-centric transform if enabled
        double axial = isFieldCentric ?
                smoothedAxial * Math.cos(heading) + smoothedLateral * Math.sin(heading) :
                smoothedAxial;
        double lateral = isFieldCentric ?
                -smoothedAxial * Math.sin(heading) + smoothedLateral * Math.cos(heading) :
                smoothedLateral;

        // Calculate motor powers for mecanum drive
        double[] powers = calculateMotorPowers(axial, lateral, smoothedYaw, speed);

        // Only update motors if powers changed to save cycles
        if (!arePowersEqual(powers, lastMotorPowers)) {
            setMotorPowers(powers);
            System.arraycopy(powers, 0, lastMotorPowers, 0, powers.length);
        }
    }

    /**
     * Handles slide control with dynamic range setting (X button),
     * preset position buttons (A, Y, B), fine adjustment with stick and D-pad,
     * and PID control to move the slide motors to the target position.
     */
    private void handleSlideControls() {
        int currentPosition = getSlidePosition();

        // Handle dynamic slide range setting with X button taps and hold
        if (gamepad2.x) {
            if (!previousX) {
                // X just pressed - record press time and increment count
                xButtonPressedTime = System.currentTimeMillis();

                xPressCount++;
                if (xPressCount == 1) {
                    // First tap sets slideMin to current position
                    slideMin = currentPosition;
                    telemetry.addData("Set Min", slideMin);
                } else if (xPressCount == 2) {
                    // Second tap sets slideMax to current position
                    slideMax = currentPosition;
                    telemetry.addData("Set Max", slideMax);
                } else {
                    // Third tap resets to defaults
                    slideMin = 150;
                    slideMax = 1300;
                    xPressCount = 0;
                    telemetry.addData("Slide Range", "Reset (Tap)");
                }
            } else {
                // X held down longer than threshold resets slide range
                if (System.currentTimeMillis() - xButtonPressedTime > xButtonHoldThreshold) {
                    slideMin = 150;
                    slideMax = 1300;
                    xPressCount = 0;
                    telemetry.addData("Slide Range", "Reset (Hold)");
                }
            }
        } else {
            // Reset press time when button released
            xButtonPressedTime = 0;
        }
        previousX = gamepad2.x;

        // Set target position to presets on buttons A (min), Y (mid), B (max)
        if (gamepad2.a) targetPosition = slideMin;
        else if (gamepad2.y) targetPosition = (slideMin + slideMax) / 2;
        else if (gamepad2.b) targetPosition = slideMax;

        // Fine adjustment using left stick Y axis
        double stickY = -gamepad2.left_stick_y;
        int fineAdjust = 10; // increments of 10 encoder counts

        if (Math.abs(stickY) > 0.05) {
            targetPosition += (int)(stickY * fineAdjust);
        }
        // Fine adjustment using D-pad up/down presses (on rising edge)
        if (gamepad2.dpad_up && !dpadUpPrev) targetPosition += fineAdjust;
        if (gamepad2.dpad_down && !dpadDownPrev) targetPosition -= fineAdjust;
        dpadUpPrev = gamepad2.dpad_up;
        dpadDownPrev = gamepad2.dpad_down;

        // Clamp target position within dynamic slide range
        targetPosition = Math.max(slideMin, Math.min(slideMax, targetPosition));

        // PID control calculation
        int error = targetPosition - currentPosition;
        integralSum = clamp(integralSum + error, -500, 500); // prevent integral windup
        double derivative = error - lastError;

        // PID coefficients tuned for your slide mechanism
        double output = (0.004 * error) + (0.0001 * integralSum) + (0.001 * derivative);
        output = clamp(output, -0.5, 0.5); // limit motor power to safe range
        lastError = error;

        // Apply calculated power to both slide motors
        leftSlide.setPower(output);
        rightSlide.setPower(output);
    }

    /**
     * Toggles the polarity of the brake power when X button is pressed on gamepad1.
     * This controls the direction of power applied during braking.
     */
    private void handleBrakePolarityToggle() {
        if (gamepad1.x && !previousX) {
            isBrakePowerNegative = !isBrakePowerNegative;
            telemetry.addData("Brake Polarity", isBrakePowerNegative ? "Negative" : "Positive");
        }
        previousX = gamepad1.x;
    }

    /**
     * Applies a brief braking power to all drive motors asynchronously.
     * Uses the polarity controlled by isBrakePowerNegative.
     */
    private void applyBrake() {
        double brakePower = isBrakePowerNegative ? -0.8 : 0.8;
        setMotorPowers(new double[]{brakePower, brakePower, brakePower, brakePower});
        sleep(500);  // Brake duration in milliseconds
        stopAllMotors();
    }

    /**
     * Stops all drive and slide motors immediately.
     */
    private void stopAllMotors() {
        setMotorPowers(new double[]{0, 0, 0, 0});
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }

    /**
     * Applies a deadzone threshold to joystick inputs to avoid drift.
     * @param val joystick value
     * @return zero if within deadzone, original value otherwise
     */
    private double applyDeadzone(double val) {
        return (Math.abs(val) < 0.05) ? 0 : val;
    }

    /**
     * Applies cubic scaling to joystick input for finer low-speed control.
     * @param val joystick value
     * @return cubed value to scale input
     */
    private double cubicScaling(double val) {
        return val * val * val;
    }

    /**
     * Calculates mecanum motor powers based on axial, lateral, and yaw inputs,
     * normalized and scaled by speed.
     * @param axial forward/backward component
     * @param lateral left/right component
     * @param yaw rotation component
     * @param speed speed scale factor
     * @return array of motor powers [leftFront, leftBack, rightFront, rightBack]
     */
    private double[] calculateMotorPowers(double axial, double lateral, double yaw, double speed) {
        double lf = axial + lateral + yaw;
        double lb = axial - lateral + yaw;
        double rf = axial - lateral - yaw;
        double rb = axial + lateral - yaw;

        // Normalize powers if any exceed 1.0
        double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(lb),
                Math.max(Math.abs(rf), Math.abs(rb)))));
        return new double[]{lf / max * speed, lb / max * speed, rf / max * speed, rb / max * speed};
    }

    /**
     * Sets the power to the four drive motors.
     * @param powers array of powers [leftFront, leftBack, rightFront, rightBack]
     */
    private void setMotorPowers(double[] powers) {
        leftFrontDrive.setPower(powers[0]);
        leftBackDrive.setPower(powers[1]);
        rightFrontDrive.setPower(powers[2]);
        rightBackDrive.setPower(powers[3]);
    }

    /**
     * Compares two arrays of motor powers to check if they are effectively equal.
     * @param a first array
     * @param b second array
     * @return true if all corresponding elements differ by less than 0.001
     */
    private boolean arePowersEqual(double[] a, double[] b) {
        for (int i = 0; i < a.length; i++) if (Math.abs(a[i] - b[i]) > 0.001) return false;
        return true;
    }

    /**
     * Gets the IMU orientation in radians with intrinsic ZYX axes order.
     * @return Orientation object containing heading info
     */
    private Orientation getIMUOrientation() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    /**
     * Returns the current heading in radians, adjusted by heading offset.
     * Inverted sign to align with field-centric convention.
     * @return heading in radians
     */
    private double getHeadingRadians() {
        return -getIMUOrientation().firstAngle + headingOffset;
    }

    /**
     * Gets the average current position of the slide motors.
     * @return average encoder count of both slide motors
     */
    private int getSlidePosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2;
    }

    /**
     * Clamps a value between min and max limits.
     * @param val value to clamp
     * @param min minimum allowable value
     * @param max maximum allowable value
     * @return clamped value
     */
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    /**
     * Updates telemetry with runtime, drive mode, heading, slide position,
     * slide range, brake polarity, and button press count.
     */
    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.1fs", runtime.seconds());
        telemetry.addData("Drive Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Heading (deg)", Math.toDegrees(getHeadingRadians()));
        telemetry.addData("Slide Pos", getSlidePosition());
        telemetry.addData("Target Pos", targetPosition);
        telemetry.addData("Slide Range", String.format("Min:%d Max:%d", slideMin, slideMax));
        telemetry.addData("Brake Polarity", isBrakePowerNegative ? "Negative" : "Positive");
        telemetry.addData("X Press Count", xPressCount);
        telemetry.update();
    }
}
