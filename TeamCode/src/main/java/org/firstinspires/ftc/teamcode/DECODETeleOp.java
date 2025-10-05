package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@TeleOp(name = "DECODETeleOp", group = "Linear OpMode")
public class DECODETeleOp extends LinearOpMode {
    // === Runtime & IMU ===
    private final ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    private double headingOffset = 0;
    private boolean isFieldCentric = true;
    // === Drive motors ===
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    // === Mechanisms ===
    private DcMotor intake, spindexer;
    private Servo kicker, hood;
    private DcMotorEx launcher;
    // === Spindexer ===
    private int spindexerStep = 0;
    private static final double TICKS_PER_STEP = 1322; // adjust CPR
    private boolean previousA = false;
    private boolean previousB = false;
    // === Launcher ===
    private double launcherRPM = 0;
    private static final double LAUNCHER_MAX_RPM = 5000;
    private static final double LAUNCHER_STEP = 250; // rpm per press
    private static final double TICKS_PER_REV = 28;  // goBILDA/REV motor, adjust if needed
    // === Hood ===
    private double hoodPos = 0.5; // mid angle
    @Override
    public void runOpMode() {
        // --- Initialize hardware ---
        initializeDrive();
        initializeIMU();
        initializeMechanisms();
        telemetry.addLine("Initialized, waiting for start...");
        telemetry.update();
        waitForStart();
        runtime.reset();
        // === Main loop ===
        while (opModeIsActive()) {
            handleDriveControls();
            handleIntakeControls();
            handleSpindexerControls();
            handleKickerControls();
            handleLauncherControls();
            handleHoodControls();
            updateTelemetry();
        }
        stopAllMotors();
    }
    // ================= DRIVE =================
    private void initializeDrive() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    private void initializeIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(params);
    }
    private void handleDriveControls() {
        // Toggle field-centric with Y
        if (gamepad1.y) isFieldCentric = !isFieldCentric;
        // Reset heading offset with B
        if (gamepad1.b) headingOffset = getHeadingRadians();
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double heading = getHeadingRadians();
        if (isFieldCentric) {
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            x = rotX; y = rotY;
        }
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double lf = (y + x + rx) / denominator;
        double lb = (y - x + rx) / denominator;
        double rf = (y - x - rx) / denominator;
        double rb = (y + x - rx) / denominator;
        leftFrontDrive.setPower(lf);
        leftBackDrive.setPower(lb);
        rightFrontDrive.setPower(rf);
        rightBackDrive.setPower(rb);
    }
    private double getHeadingRadians() {
        Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return -o.firstAngle + headingOffset;
    }
    // ================= MECHANISMS =================
    private void initializeMechanisms() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        spindexer = hardwareMap.get(DcMotor.class, "spindexer");
        kicker = hardwareMap.get(Servo.class, "kicker");
        hood = hardwareMap.get(Servo.class, "hood");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void handleIntakeControls() {
        if (gamepad2.right_trigger > 0.1) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0.0);
        }
    }
    private void handleSpindexerControls() {
        boolean aPressed = gamepad2.a;
        if (aPressed && !previousA) {
            spindexerStep = (spindexerStep + 1) % 3;
            int targetPos = (int)(spindexerStep * TICKS_PER_STEP);
            spindexer.setTargetPosition(targetPos);
            spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexer.setPower(0.6);
            while (opModeIsActive() && spindexer.isBusy()) {
                telemetry.addData("Spindexer", "Moving to step %d", spindexerStep);
                telemetry.update();
            }
            spindexer.setPower(0);
            spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        previousA = aPressed;
    }
    private void handleKickerControls() {
        boolean bPressed = gamepad2.b;
        if (bPressed && !previousB) {
            kicker.setPosition(1.0); // extend
            sleep(250);
            kicker.setPosition(0.0); // retract
        }
        previousB = bPressed;
    }
    private void handleLauncherControls() {
        if (gamepad2.right_bumper) {
            launcherRPM = Math.min(launcherRPM + LAUNCHER_STEP, LAUNCHER_MAX_RPM);
        }
        if (gamepad2.left_bumper) {
            launcherRPM = Math.max(launcherRPM - LAUNCHER_STEP, 0);
        }
        double ticksPerSec = (launcherRPM / 60.0) * TICKS_PER_REV;
        launcher.setVelocity(ticksPerSec);
    }
    private void handleHoodControls() {
        double stick = -gamepad2.left_stick_y; // up is positive
        hoodPos += stick * 0.01; // scale adjust speed
        hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
        hood.setPosition(hoodPos);
    }
    private void stopAllMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        intake.setPower(0);
        spindexer.setPower(0);
        launcher.setPower(0);
    }
    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.1fs", runtime.seconds());
        telemetry.addData("Drive Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Heading (deg)", Math.toDegrees(getHeadingRadians()));
        telemetry.addData("Spindexer Step", spindexerStep);
        telemetry.addData("Spindexer Pos", spindexer.getCurrentPosition());
        telemetry.addData("Launcher RPM", "%.0f", launcherRPM);
        telemetry.addData("Hood Pos", "%.2f", hoodPos);
        telemetry.update();
    }
}