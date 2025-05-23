package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * An advanced TeleOp OpMode that supports:
 * - Field-centric driving using IMU
 * - Robot-centric and field-centric mode switching via Y button (with debounce)
 * - Threaded braking with ABS simulation
 * - Smoothed joystick inputs
 * - Dynamic speed control with right trigger
 */
@TeleOp(name = "Dynamic Thread Field-Centric Omni OpMode", group = "Linear OpMode")
public class ImprovedTeleOpTEST extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private BNO055IMU imu;
    private double headingOffset = 0;
    private boolean isFieldCentric = true;

    private double smoothedAxial = 0.0;
    private double smoothedLateral = 0.0;
    private double smoothedYaw = 0.0;

    private final ExecutorService executor = Executors.newCachedThreadPool();
    private final AtomicBoolean isBraking = new AtomicBoolean(false);

    private final double[] lastMotorPowers = new double[] {0, 0, 0, 0};

    private static final double DEADBAND = 0.05;
    private static final double SMOOTHING_FACTOR = 0.15;
    private static final int BRAKE_DURATION_MS = 500;
    private static final double BRAKE_MAX_INTENSITY = -0.8;

    // For button debounce
    private boolean previousB = false;
    private boolean previousY = false;

    @Override
    public void runOpMode() {
        initializeHardware();
        initializeIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Debounced B button: reset heading offset
            boolean currentB = gamepad1.b;
            if (currentB && !previousB) {
                headingOffset = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
                telemetry.addData("IMU", "Heading reset");
                telemetry.update();
            }
            previousB = currentB;

            // Debounced Y button: toggle field-centric mode
            boolean currentY = gamepad1.y;
            if (currentY && !previousY) {
                isFieldCentric = !isFieldCentric;
            }
            previousY = currentY;

            // Braking with left trigger, in separate thread to avoid blocking
            if (gamepad1.left_trigger > 0.1 && isBraking.compareAndSet(false, true)) {
                executor.submit(() -> {
                    applyBrake();
                    isBraking.set(false);
                });
            }

            double rawAxial = applyDeadzone(-gamepad1.left_stick_y);
            double rawLateral = applyDeadzone(gamepad1.left_stick_x);
            double rawYaw = applyDeadzone(gamepad1.right_stick_x);

            updateJoystickInputs(rawAxial, rawLateral, rawYaw);

            double speedFactor = (gamepad1.right_trigger > 0.1) ? 0.4 : 1.0;

            double heading = getHeadingRadians();
            double rotatedAxial = isFieldCentric ?
                    smoothedAxial * Math.cos(heading) - smoothedLateral * Math.sin(heading) :
                    smoothedAxial;
            double rotatedLateral = isFieldCentric ?
                    smoothedAxial * Math.sin(heading) + smoothedLateral * Math.cos(heading) :
                    smoothedLateral;

            double[] powers = calculateMotorPowers(rotatedAxial, rotatedLateral, smoothedYaw, speedFactor);

            if (!arePowersEqual(powers, lastMotorPowers)) {
                setMotorPowers(powers);
                System.arraycopy(powers, 0, lastMotorPowers, 0, powers.length);
            }

            telemetry.addData("Heading (deg)", Math.toDegrees(heading));
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", powers[0], powers[1]);
            telemetry.addData("Back left/Right", "%4.2f, %4.2f", powers[2], powers[3]);
            telemetry.addData("Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

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
    }

    private void initializeIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = null;

        imu.initialize(parameters);

        telemetry.addData("IMU", "Calibrating... Please wait");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("IMU", "Calibration complete");
        telemetry.update();
    }

    private double getHeadingRadians() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return -(orientation.firstAngle - headingOffset);
    }

    private double applyDeadzone(double value) {
        return (Math.abs(value) > DEADBAND) ? value : 0.0;
    }

    private double[] calculateMotorPowers(double axial, double lateral, double yaw, double speedFactor) {
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        return new double[] {
                leftFrontPower * speedFactor,
                rightFrontPower * speedFactor,
                leftBackPower * speedFactor,
                rightBackPower * speedFactor
        };
    }

    private synchronized void setMotorPowers(double[] powers) {
        leftFrontDrive.setPower(clamp(powers[0]));
        rightFrontDrive.setPower(clamp(powers[1]));
        leftBackDrive.setPower(clamp(powers[2]));
        rightBackDrive.setPower(clamp(powers[3]));
    }

    private void updateJoystickInputs(double targetAxial, double targetLateral, double targetYaw) {
        smoothedAxial += SMOOTHING_FACTOR * (targetAxial - smoothedAxial);
        smoothedLateral += SMOOTHING_FACTOR * (targetLateral - smoothedLateral);
        smoothedYaw += SMOOTHING_FACTOR * (targetYaw - smoothedYaw);
    }

    private boolean arePowersEqual(double[] p1, double[] p2) {
        for (int i = 0; i < p1.length; i++) {
            if (Math.abs(p1[i] - p2[i]) > 0.01) return false;
        }
        return true;
    }

    /**
     * Apply anti-lock braking with pulsed motor brake power scaled by speed.
     */
    private void applyBrake() {
        double currentSpeed = calculateCurrentSpeed();
        double brakePower = clamp(BRAKE_MAX_INTENSITY * currentSpeed);

        long pulseInterval = 100; // ms between pulses
        long pulseDuration = 50;  // ms brake on duration

        long endTime = System.currentTimeMillis() + BRAKE_DURATION_MS;

        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            setMotorPowers(new double[] {brakePower, brakePower, brakePower, brakePower});
            sleep(pulseDuration);
            setMotorPowers(new double[] {0.0, 0.0, 0.0, 0.0});
            sleep(pulseInterval - pulseDuration);
        }
    }

    /**
     * Calculate current robot speed based on motor powers (normalized 0-1)
     */
    private double calculateCurrentSpeed() {
        double totalSpeed = Math.sqrt(
                Math.pow(leftFrontDrive.getPower(), 2) +
                        Math.pow(leftBackDrive.getPower(), 2) +
                        Math.pow(rightFrontDrive.getPower(), 2) +
                        Math.pow(rightBackDrive.getPower(), 2));
        return totalSpeed / 2.0;  // max sqrt(4) = 2, so normalize by 2
    }

    /**
     * Clamp motor power values to [-1, 1].
     */
    private double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}
