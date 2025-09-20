package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleopDrivetrain {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public IMU imu;



    private ElapsedTime runtime = new ElapsedTime();

    LinearOpMode opMode;

    public TeleopDrivetrain(LinearOpMode op) {
        opMode = op;
    }

    public void initDriveTrain(HardwareMap hardwareMap) {
        // imu tomfoolery
//        imu = hardwareMap.get(IMU.class, "imu"); // Match the name in the configuration
//        IMU.Parameters parameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,  // Logo facing backward
//                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD         // USB ports facing up
//                )
//        );
//        imu.initialize(parameters);

        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backLeft= hardwareMap.get(DcMotor.class, "leftBack");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        frontRight.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        backLeft.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        backRight.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void stopMotors(){
        opMode.telemetry.addData("Status", "Stopped");
        opMode.telemetry.update();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveForward(double power, long targetInSeconds) {
        runtime.reset();

        opMode.telemetry.addData("Status", "Moving Forward");
        opMode.telemetry.update();

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        opMode.sleep(targetInSeconds);
    }

    public void moveBackwards(double power, long targetInMillis) {
        opMode.telemetry.addData("Status", "Moving Backward");
        opMode.telemetry.update();

        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);

        while(runtime.milliseconds() < targetInMillis) {
            //Keeps on looping until target is reached
        }
        stopMotors();
    }

    public void strafeLeft(double power, long targetTimeMillis) {
        runtime.reset();

        opMode.telemetry.addData("Status", "Moving Left");
        opMode.telemetry.update();

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);

        while(runtime.milliseconds() < targetTimeMillis) {
            //keeps on looping until target is reached
        }
        stopMotors();

    }

    public void strafeRight(double power, long targetTimeMillis){
        runtime.reset();

        opMode.telemetry.addData("Status", "Moving Right");
        opMode.telemetry.update();

        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);

        while(runtime.milliseconds() < targetTimeMillis) {

        }
        stopMotors();
    }

    public void rotate(double power, long targetInMilis) {
        opMode.telemetry.addData("Status", "Rotating");
        opMode.telemetry.update();

        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);

        opMode.sleep(targetInMilis);
    }

    public void rotateRight(double power, long targetInMilis) {
        opMode.telemetry.addData("Status", "Rotating");
        opMode.telemetry.update();

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);

        opMode.sleep(targetInMilis);
    }

    public void rotateToAngle(double targetAngle, double power) {
        double tolerance = 1.0; // Tolerance in degrees for accuracy
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw();

        // Normalize target angle to be within 0-360 range
        targetAngle = ((targetAngle % 360) + 360) % 360;

        // Loop until within tolerance of target angle
        while (opMode.opModeIsActive()) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw();

            // Calculate the difference between current and target angles
            double angleDiff = (targetAngle - currentAngle + 360) % 360;

            // If the angle difference is greater than 180, it's shorter to rotate in the opposite direction
            if (angleDiff > 180) {
                angleDiff -= 360; // Rotate counterclockwise instead of clockwise
            }

            // Check if we're within the target tolerance
            if (Math.abs(angleDiff) < tolerance) {
                break;
            }

            // Determine the rotation direction based on angle difference
            double direction = angleDiff > 0 ? 1 : -1; // Clockwise if positive, counterclockwise if negative

            // Rotate with the calculated power and direction
            rotate(direction * power, 50); // Rotate for a small increment (e.g., 50 ms)

            // Telemetry for debugging
            opMode.telemetry.addData("Target Angle", targetAngle);
            opMode.telemetry.addData("Current Angle", currentAngle);
            opMode.telemetry.addData("Angle Difference", angleDiff);
            opMode.telemetry.update();
        }

        // Stop the motors once the target angle is reached
        stopMotors();
    }

//    private int convertInchesToTicks(double inches) {
//        double ticksPerRevolution = 1538; // Example: Neverest 20 motor
//        double wheelCircumference = 4 * Math.PI; // 4-inch diameter wheel
//        return (int) ((inches / wheelCircumference) * ticksPerRevolution);
//    }
//
//    public void moveForward(double distanceInInches, double power) {
//        int targetTicks = convertInchesToTicks(distanceInInches);
//        //resetEncoders();
//
//
//        frontLeft.setPower(power);
//        frontRight.setPower(power);
//        backLeft.setPower(power);
//        backRight.setPower(power); // Forward power
//        while (Math.abs(getEncoderPosition()) < targetTicks) {
//            // Wait until the target is reached
//        }
//        stopMotors();
//    }





   /*
    public void initGyuro(HardwareMap hardwareMap) {
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu = hardwareMap.get(IMU.class, "imu");
        ypr = imu.getRobotYawPitchRollAngles();
        imu.initialize(parameters);


    }

    */

    public void presetL2() {
    }

    public void presetR2() {

    }
}

