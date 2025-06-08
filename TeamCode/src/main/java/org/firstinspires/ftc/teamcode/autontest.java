package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "SlideExtendRetractDriveTurn", group = "Test")
public class autontest extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private BNO055IMU imu;

    private static final int slideMin = 0;
    private static final int slideMax = 1650;

    private int targetPosition = 0;

    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;

    @Override
    public void runOpMode() {
        // Hardware map
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);  // reverse left motors
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reversed slide motor directions
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        // Drive motors run mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Slide motors encoder reset and run using encoder
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Extend slides
        targetPosition = slideMax;
        waitForSlideAtTarget();
        sleep(1000);

        // Retract slides
        targetPosition = slideMin;
        waitForSlideAtTarget();
        sleep(1000);

        // Drive forward 1 second
        driveStraight(0.5);

        // Drive backward 1 second
        driveStraight(-0.5);

        // Full 360 degree turn clockwise
        turn360Degrees();

        stopAllMotors();

        telemetry.addData("Status", "Done");
        telemetry.update();
        sleep(2000);
    }

    private void driveStraight(double power) {
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < 1000) {  // 1 second drive
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);

            // Maintain slide position during drive
            handleSlideControl();

            telemetry.addData("Drive Power", power);
            telemetry.update();
            idle();
        }

        stopDriveMotors();
    }

    private void handleSlideControl() {
        int currentPosLeft = leftSlide.getCurrentPosition();
        int currentPosRight = rightSlide.getCurrentPosition();

        // Clamp targetPosition within limits
        targetPosition = (int) clamp(targetPosition, slideMin, slideMax);

        // Average position and error
        double currentPosAvg = (currentPosLeft + currentPosRight) / 2.0;
        double error = targetPosition - currentPosAvg;

        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0; // seconds

        if (lastTime == 0) {
            // First run: initialize lastTime and lastError
            lastTime = currentTime;
            lastError = error;
            return;  // skip control this cycle to get deltaTime next time
        }

        if (deltaTime > 0) {
            // Integral with anti-windup: only integrate if error is small or power not saturated
            double kP = 0.0008;
            double kI = 0.0001; // small integral gain, tune as needed
            double kD = 0.0002; // small derivative gain, tune as needed

            // Calculate derivative
            double derivative = (error - lastError) / deltaTime;

            // Update integral with clamping for anti-windup
            integral += error * deltaTime;
            integral = clamp(integral, -1000, 1000); // arbitrary clamp limits to prevent windup

            // PID output
            double power = kP * error + kI * integral + kD * derivative;

            // Clamp power to motor range
            power = clamp(power, -1, 1);

            // Prevent pushing beyond physical limits
            if ((currentPosAvg >= slideMax && power > 0) || (currentPosAvg <= slideMin && power < 0)) {
                power = 0;
                integral = 0; // reset integral if at limit to prevent windup
            }

            leftSlide.setPower(power);
            rightSlide.setPower(power);

            lastError = error;
            lastTime = currentTime;

            telemetry.addData("Slide Left Pos", currentPosLeft);
            telemetry.addData("Slide Right Pos", currentPosRight);
            telemetry.addData("Slide Target", targetPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.addData("Integral", integral);
            telemetry.addData("Derivative", derivative);
            telemetry.addData("DeltaTime (s)", deltaTime);
            telemetry.update();
        }
    }

    private void waitForSlideAtTarget() {
        int threshold = (int)((slideMax - slideMin) * 0.05);  // 5% tolerance
        lastTime = 0;   // Reset timing and error for PID controller
        integral = 0;

        while (opModeIsActive()) {
            int currentPosLeft = leftSlide.getCurrentPosition();
            int currentPosRight = rightSlide.getCurrentPosition();
            int averagePos = (currentPosLeft + currentPosRight) / 2;
            int error = Math.abs(targetPosition - averagePos);

            handleSlideControl();

            telemetry.addData("Slide Left Pos", currentPosLeft);
            telemetry.addData("Slide Right Pos", currentPosRight);
            telemetry.addData("Slide Target", targetPosition);
            telemetry.addData("Error", error);
            telemetry.update();

            if (error <= threshold) {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
                break;
            }
            idle();
        }
    }

    private void turn360Degrees() {
        double startHeading = getHeadingRadians();
        double targetHeading = normalizeAngle(startHeading + 2 * Math.PI);

        double tolerance = Math.toRadians(2); // 2 degrees tolerance

        while (opModeIsActive()) {
            double currentHeading = getHeadingRadians();
            double error = angleDifference(targetHeading, currentHeading);

            if (Math.abs(error) < tolerance) break;

            // Proportional turning power
            double turnPower = clamp(0.8 * error, -0.6, 0.6);

            leftFrontDrive.setPower(turnPower);
            leftBackDrive.setPower(turnPower);
            rightFrontDrive.setPower(-turnPower);
            rightBackDrive.setPower(-turnPower);

            // Maintain slide position during turn
            handleSlideControl();

            telemetry.addData("Heading (deg)", Math.toDegrees(currentHeading));
            telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
            telemetry.addData("Turn Error (deg)", Math.toDegrees(error));
            telemetry.update();

            idle();
        }
        stopDriveMotors();
    }

    private double getHeadingRadians() {
        return normalizeAngle(imu.getAngularOrientation().firstAngle);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }

    private void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void stopAllMotors() {
        stopDriveMotors();
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
