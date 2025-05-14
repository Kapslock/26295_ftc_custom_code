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
 * - Robot-centric and field-centric mode switching via Y button
 * - Threaded braking
 * - Smoothed joystick inputs
 * - Dynamic speed control with right trigger
 * - Anti-lock braking system (ABS) implementation
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
    private boolean isFieldCentric = true;  // Initial mode is Field-Centric

    private double smoothedAxial = 0.0;
    private double smoothedLateral = 0.0;
    private double smoothedYaw = 0.0;

    private final ExecutorService executor = Executors.newCachedThreadPool();
    private final AtomicBoolean isBraking = new AtomicBoolean(false);

    private final double[] lastMotorPowers = new double[] {0, 0, 0, 0};

    private static final double DEADBAND = 0.05;
    private static final double SMOOTHING_FACTOR = 0.15;
    private static final int BRAKE_DURATION_MS = 500;
    private static final double BRAKE_MAX_INTENSITY = -0.8;  // Maximum brake intensity (scaled for ABS)

    @Override
    public void runOpMode() {
        initializeHardware();
        initializeIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                headingOffset = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            }

            if (gamepad1.left_trigger > 0.1 && isBraking.compareAndSet(false, true)) {
                executor.submit(() -> {
                    applyBrake();
                    isBraking.set(false);
                });
            }

            // Swap between field-centric and robot-centric on Y button press
            if (gamepad1.y) {
                isFieldCentric = !isFieldCentric;
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
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
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

    private void initializeIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
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
        leftFrontDrive.setPower(powers[0]);
        rightFrontDrive.setPower(powers[1]);
        leftBackDrive.setPower(powers[2]);
        rightBackDrive.setPower(powers[3]);
    }

    private void updateJoystickInputs(double targetAxial, double targetLateral, double targetYaw) {
        smoothedAxial += SMOOTHING_FACTOR * (targetAxial - smoothedAxial);
        smoothedLateral += SMOOTHING_FACTOR * (targetLateral - smoothedLateral);
        smoothedYaw += SMOOTHING_FACTOR * (targetYaw - smoothedYaw);
    }
// when i wrote lines 190-195, only god and i knew how they worked. now only god knows. so please update the following counter as needed
    //hours wasted here: 1
    private boolean arePowersEqual(double[] p1, double[] p2) {
        for (int i = 0; i < p1.length; i++) {
            if (Math.abs(p1[i] - p2[i]) > 0.01) return false;
        }
        return true;
    }

    /**
     * Calculates and applies anti-lock braking to prevent wheel lockup.
     * The brake power is applied in pulses to reduce the chances of wheel lock.
     */
    private void applyBrake() {
        double currentSpeed = calculateCurrentSpeed();
        double brakePower = BRAKE_MAX_INTENSITY * currentSpeed;  // Scale the brake power by current speed

        // Pulsed braking logic to simulate anti-lock braking system
        long pulseInterval = 100; // milliseconds between brake power pulses
        long pulseDuration = 50;  // milliseconds for each brake pulse

        long pulseEndTime = (long) (runtime.milliseconds() + pulseInterval);

        while (runtime.milliseconds() < pulseEndTime) {
            setMotorPowers(new double[] {brakePower, brakePower, brakePower, brakePower});
            sleep(pulseDuration);
            setMotorPowers(new double[] {0.0, 0.0, 0.0, 0.0});
            sleep(pulseInterval - pulseDuration); // Allow for a delay before next pulse
        }
    }

    /**
     * Calculate the current speed of the robot based on the motor powers.
     * This will give a magnitude of the velocity based on the current motor power values.
     * @return The robot's speed (0 to 1).
     */
    private double calculateCurrentSpeed() {
        double totalSpeed = Math.sqrt(Math.pow(leftFrontDrive.getPower(), 2) +
                Math.pow(leftBackDrive.getPower(), 2) +
                Math.pow(rightFrontDrive.getPower(), 2) +
                Math.pow(rightBackDrive.getPower(), 2));
        return totalSpeed / 4.0;  // Normalize the speed to a scale from 0 to 1
    }
}
// end