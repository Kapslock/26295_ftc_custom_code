package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// OpMode FTC SIM ONLY
// ATTENTION : nommage obligatoire pour ftcsim
// manque la @Teleop pour un vrai TeamCode
public class MyFIRSTJavaOpMode extends LinearOpMode {
    final double DESIRED_DISTANCE = 25.0; //  this is how close the camera should get to the target (inches)

    // TODO : enum not working with ftc sim
    static final int BLUE_TAG_ID = 20;
    static final int RED_TAG_ID = 24;
    static final int CENTER_TAG_ID = 21;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;

    private void initialize() {
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        // imu = hardwareMap.get(IMU.class, "imu"); not available sur ftc sim

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("initialize", "done");
        telemetry.update();
    }


    // TODO obligé de tenir les builders, obligé d'avois vision portal sinon pas de datas
    VisionPortal.Builder myVisionPortalBuilder;
    VisionPortal myVisionPortal;
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    AprilTagProcessor myApriltagProcessor;

    private void initializeVisionPortal() {
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortal = (myVisionPortalBuilder.build());
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myApriltagProcessor = (myAprilTagProcessorBuilder.build());
        myVisionPortalBuilder.addProcessor(myApriltagProcessor);
    }

    private void processDriveInputs(double turn, double forward, double strafe) {
        // Combine inputs to create drive and turn (or both!)
        double frontLeftPower = forward + turn + strafe;
        double frontRightPower = forward + turn - strafe;
        double backLeftPower = forward - turn + strafe;
        double backRightPower = forward + turn + strafe;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

//        telemetry.addData("processDriveInputs", "turn:" + turn + " forward:" + forward + " strafe:" + strafe);
//        telemetry.addData("frontLeftPower", frontLeftPower);
//        telemetry.addData("frontRightPower: ", frontRightPower);
//        telemetry.addData("backLeftPower: ", backLeftPower);
//        telemetry.addData("max: ", max);
//        telemetry.update();

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
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

    public void demoDrive() {
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
        slideRightFor(1000);

        keyboardDrive();
    }


    private void keyboardDrive() {
        // debrider ca,
        // permettre de lancer autodrive sur une touche,
        // de l'arreter sun une autre
        // au depart on est en mode keyboard et on choisit de lancer l'autodrive
        TODO
        while (opModeIsActive()) {
            float turn = keyboard.isPressed(108) - keyboard.isPressed(106); // j et l
            float forward = keyboard.isPressed(105) - keyboard.isPressed(107); // i et k
            float strafe = keyboard.isPressed(111) - keyboard.isPressed(117); // o et u
            processDriveInputs(turn, forward, strafe);
            displayVisionPortalData();
        }
    }

    private void displayVisionPortalData() {
        for (AprilTagDetection myAprilTagDetection : myApriltagProcessor.getDetections()) {
            telemetry.addData("ID", (myAprilTagDetection.id));
            telemetry.addData("Range", (myAprilTagDetection.ftcPose.range));
            telemetry.addData("Yaw", (myAprilTagDetection.ftcPose.yaw));
            telemetry.addData("Bearing", (myAprilTagDetection.ftcPose.bearing));
        }
        telemetry.update();
    }

    private void autoDrive() {
        boolean autoDriveMode = false;
        while (!autoDriveMode) {
            displayVisionPortalData();
            boolean found = false;
            for (AprilTagDetection myAprilTagDetection : myApriltagProcessor.getDetections()) {
                if (myAprilTagDetection.id == 20) {
                    found = true;
//                    telemetry.addData("Found", "ID %d (%s)", myAprilTagDetection.id, myAprilTagDetection.metadata.name);
//                    telemetry.addData("Range", "%5.1f inches", myAprilTagDetection.ftcPose.range);
//                    telemetry.addData("Bearing", "%3.0f degrees", myAprilTagDetection.ftcPose.bearing);
//                    telemetry.addData("Yaw", "%3.0f degrees", myAprilTagDetection.ftcPose.yaw);
//                    telemetry.update();
                    telemetry.addData("ID", (myAprilTagDetection.id));
                    telemetry.addData("Range", (myAprilTagDetection.ftcPose.range));
                    telemetry.addData("Yaw", (myAprilTagDetection.ftcPose.yaw));
                    telemetry.addData("Bearing", (myAprilTagDetection.ftcPose.bearing));
                    telemetry.update();

                    if (myAprilTagDetection.ftcPose.range < DESIRED_DISTANCE && myAprilTagDetection.ftcPose.bearing == 0 && myAprilTagDetection.ftcPose.yaw == 0) {
                        autoDriveMode = true;
                        telemetry.addData("END", "END");
                        telemetry.update();
                        break;
                    }

                    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
                    //  applied to the drive motors to correct the error.
                    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
                    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
                    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
                    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

                    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
                    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
                    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

                    double rangeError = (myAprilTagDetection.ftcPose.range - DESIRED_DISTANCE);
                    double headingError = myAprilTagDetection.ftcPose.bearing;
                    double yawError = myAprilTagDetection.ftcPose.yaw;
                    telemetry.addData("auto", "rangeError:" + rangeError + "- headingError:" + headingError + "- yawError:" + yawError);
                    telemetry.update();
                    // Use the speed and turn "gains" to calculate how we want the robot to move.
//                    double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//                    double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//                    double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    double drive = clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    double turn = clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    double strafe = clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    telemetry.addData("autodrive", "drive:" + drive + " turn:" + turn + " strafe:" + strafe);
                    telemetry.update();
                    processDriveInputs(turn, drive, strafe);
                    sleep(10);
                    stopWheels();
                    break;
                }
            }
            if (!found) {
                turnLeftFor(10);
                sleep(50);
                displayVisionPortalData();
                sleep(1000);
                // TODO mettre un garde fou apres un tour complet?
            }
        }

    }

    private double clip(double value, double min, double max) {
        if (value < min) {
            return min;
        } else if (value > max) {
            return max;
        }
        return value;
    }


    @Override
    public void runOpMode() {
        initialize();
        initializeVisionPortal();
        waitForStart();

        // demoDrive();
        autoDrive();
        //keyboardDrive();

    }


}