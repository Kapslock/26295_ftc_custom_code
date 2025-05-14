package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(name = "Dynamic Thread Field-Centric Omni OpMode", group = "Linear OpMode")
public class ImprovedTeleOpTEST extends LinearOpMode {

    // Runtime for measuring elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    // Motor declarations for the four wheels
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // IMU (Inertial Measurement Unit) for field-centric movement
    private BNO055IMU imu;
    private double headingOffset = 0;  // Offsets heading when recalibration occurs

    // Smoothing variables for gradual joystick response
    private double smoothedAxial = 0.0;
    private double smoothedLateral = 0.0;
    private double smoothedYaw = 0.0;
    private final double SMOOTHING_FACTOR = 0.15;  // Factor to control the smoothness of movement

    // Thread pool for handling dynamic process execution (motor control and braking)
    private final ExecutorService executor = Executors.newCachedThreadPool();
    private final AtomicInteger activeThreads = new AtomicInteger(0);  // Track number of active threads

    @Override
    public void runOpMode() {
        // Initialize hardware and IMU
        initializeHardware();
        initializeIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait until the Start button is pressed
        waitForStart();
        runtime.reset();  // Reset the runtime timer

        // Main loop runs as long as the op mode is active
        while (opModeIsActive()) {

            // If B button is pressed, reset heading offset to the current orientation
            if (gamepad1.b) {
                headingOffset = imu.getAngularOrientation().firstAngle;
            }

            // **Brake Mode**: Triggered when Left Trigger is pressed
            if (gamepad1.left_trigger > 0.1) {
                spawnBrakeThread();  // Apply braking mechanism in a separate thread
                continue;  // Skip the rest of the loop and focus on braking
            }

            // **Joystick Input Handling**: Capture raw inputs from the joysticks
            double targetAxial = applyDeadzone(-gamepad1.left_stick_y);  // Forward/Backward (Y-axis)
            double targetLateral = applyDeadzone(gamepad1.left_stick_x);  // Strafe Left/Right (X-axis)
            double targetYaw = applyDeadzone(gamepad1.right_stick_x);  // Rotation (X-axis of right stick)

            // Only update the smoothed inputs if there is a change in the joystick values
            if (targetAxial != smoothedAxial || targetLateral != smoothedLateral || targetYaw != smoothedYaw) {
                updateJoystickInputs(targetAxial, targetLateral, targetYaw);
            }

            // **Slow Mode**: Reduce speed if Right Trigger is pressed
            double speedFactor = (gamepad1.right_trigger > 0.1) ? 0.4 : 1.0;

            // **Field-Centric Transformation**: Adjust the movement to be relative to the field orientation
            double heading = getHeadingRadians();  // Get the current heading in radians
            double rotatedAxial = smoothedAxial * Math.cos(heading) - smoothedLateral * Math.sin(heading);  // Apply rotation
            double rotatedLateral = smoothedAxial * Math.sin(heading) + smoothedLateral * Math.cos(heading);  // Apply rotation

            // Calculate motor powers for each wheel
            double[] powers = calculateMotorPowers(rotatedAxial, rotatedLateral, smoothedYaw, speedFactor);

            // Dynamically spawn threads for each motor control process
            spawnMotorControlThreads(powers);

            // Telemetry data for debugging and visual feedback in the driver station
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Heading (deg)", Math.toDegrees(heading));
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", powers[0], powers[1]);
            telemetry.addData("Back left/Right", "%4.2f, %4.2f", powers[2], powers[3]);
            telemetry.update();
        }
    }

    // ----------------------- Initialization -----------------------

    /**
     * Initializes hardware components such as motors.
     */
    private void initializeHardware() {
        // Motor hardware setup
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Initializes the IMU (Inertial Measurement Unit) for field-centric movement.
     */
    private void initializeIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;  // Use radians for angle measurement
        imu.initialize(parameters);

        // Wait for IMU to be calibrated
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
    }
    // ----------------------- Helpers -----------------------

    /**
     * Gets the current heading of the robot in radians.
     * @return Heading in radians adjusted by the heading offset.
     */
    private double getHeadingRadians() {
        Orientation orientation = imu.getAngularOrientation();
        return -(orientation.firstAngle - headingOffset);  // Inverted to match expected direction
    }

    /**
     * Applies a deadzone to joystick inputs to prevent small movements from triggering.
     * @param value Joystick input value.
     * @return Adjusted value with deadzone applied.
     */
    private double applyDeadzone(double value) {
        return (Math.abs(value) > 0.05) ? value : 0.0;  // Apply deadzone of 5%
    }

    /**
     * Calculates the power for each motor based on the axial, lateral, and yaw inputs.
     * Normalizes motor powers to prevent overspeed.
     * @param axial Forward/backward movement.
     * @param lateral Strafe left/right movement.
     * @param yaw Rotation input.
     * @param speedFactor Speed scaling factor (based on slow mode).
     * @return Array of motor powers for each wheel.
     */
    private double[] calculateMotorPowers(double axial, double lateral, double yaw, double speedFactor) {
        // Compute motor power for each wheel
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize motor powers if they exceed 1.0 to prevent motor overdrive
        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Apply the speed factor (slow mode)
        return new double[] {
                leftFrontPower * speedFactor,
                rightFrontPower * speedFactor,
                leftBackPower * speedFactor,
                rightBackPower * speedFactor
        };
    }

    /**
     * Sets the power for all motors based on the calculated powers.
     * @param powers Array of motor powers.
     */
    private void setMotorPowers(double[] powers) {
        leftFrontDrive.setPower(powers[0]);
        rightFrontDrive.setPower(powers[1]);
        leftBackDrive.setPower(powers[2]);
        rightBackDrive.setPower(powers[3]);
    }

    /**
     * Spawns separate threads to control each motor independently.
     * This allows for parallel motor control and can be adjusted dynamically.
     * @param powers Array of motor powers.
     */
    private void spawnMotorControlThreads(double[] powers) {
        // Increment thread count and assign each motor control to a new thread
        activeThreads.set(0);
        executor.submit(() -> {
            leftFrontDrive.setPower(powers[0]);
            activeThreads.incrementAndGet();  // Track this thread as active
        });
        executor.submit(() -> {
            rightFrontDrive.setPower(powers[1]);
            activeThreads.incrementAndGet();
        });
        executor.submit(() -> {
            leftBackDrive.setPower(powers[2]);
            activeThreads.incrementAndGet();
        });
        executor.submit(() -> {
            rightBackDrive.setPower(powers[3]);
            activeThreads.incrementAndGet();
        });
    }

    /**
     * Spawns a thread to apply braking to the robot by temporarily reversing the motors.
     */
    private void spawnBrakeThread() {
        executor.submit(() -> {
            applyBrake();  // Apply brake
            activeThreads.incrementAndGet();  // Track brake thread as active
        });
    }

    /**
     * Smoothly updates the joystick inputs to reduce abrupt movements.
     * @param targetAxial New axial input (forward/backward).
     * @param targetLateral New lateral input (strafe).
     * @param targetYaw New yaw input (rotation).
     */
    private void updateJoystickInputs(double targetAxial, double targetLateral, double targetYaw) {
        smoothedAxial += SMOOTHING_FACTOR * (targetAxial - smoothedAxial);
        smoothedLateral += SMOOTHING_FACTOR * (targetLateral - smoothedLateral);
        smoothedYaw += SMOOTHING_FACTOR * (targetYaw - smoothedYaw); // Fix missing assignment for yaw
    }

    /**
     * Applies a braking action to the robot by temporarily reversing motor powers.
     */
    private void applyBrake() {
        // Temporarily reverse all motors for braking effect
        leftFrontDrive.setPower(-0.5);
        rightFrontDrive.setPower(-0.5);
        leftBackDrive.setPower(-0.5);
        rightBackDrive.setPower(-0.5);

        sleep(500);  // Apply brake for 500ms

        // Stop the motors after braking
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
