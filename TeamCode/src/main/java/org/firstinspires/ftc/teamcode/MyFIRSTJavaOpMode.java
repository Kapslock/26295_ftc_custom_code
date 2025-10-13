import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

// OpMode FTC SIM ONLY
// ATTENTION : nommage obligatoire pour ftcsim
// manque la @Teleop pour un vrai TeamCode
public class MyFIRSTJavaOpMode extends LinearOpMode {
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    IMU imu;

    private void initialize() {
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("init", "done");
        telemetry.update();
    }


    private void keyboardDrive() {
        while (opModeIsActive()) {
            float turn = keyboard.isPressed(108) - keyboard.isPressed(106); // j et l
            float forward = keyboard.isPressed(105) - keyboard.isPressed(107); // i et k
            float strafe = keyboard.isPressed(111) - keyboard.isPressed(117); // o et u
            processDriveInputs(turn, forward, strafe);
        }
    }

    private void processDriveInputs(float turn, float forward, float strafe) {
        // Combine inputs to create drive and turn (or both!)
        float frontLeftPower = forward + turn + strafe;
        float frontRightPower = forward + turn - strafe;
        float backLeftPower = forward - turn + strafe;
        float backRightPower = forward + turn + strafe;
        telemetry.addData("processDriveInputs", "turn:" + turn + " forward:" + forward + " strafe:" + strafe);
        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("frontRightPower: ", frontRightPower);
        telemetry.addData("backLeftPower: ", backLeftPower);
        telemetry.addData("backRightPower: ", backRightPower);
        telemetry.update();
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


    public void autoDrive() {
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

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        //keyboardDrive();
        autoDrive();

    }


}