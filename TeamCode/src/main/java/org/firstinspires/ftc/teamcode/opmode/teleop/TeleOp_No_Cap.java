package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.component.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.component.mechanism.Shooter;
import org.firstinspires.ftc.teamcode.component.sensor.GoBildaPinpointDriver;

@TeleOp(name="TeleOp_No_Cap", group="Production")
public class TeleOp_No_Cap extends OpMode
{
    final String FRONT_LEFT_DRIVE_MOTOR_NAME = "front_left";
    final String FRONT_RIGHT_DRIVE_MOTOR_NAME = "front_right";
    final String REAR_LEFT_DRIVE_MOTOR_NAME = "rear_left";
    final String REAR_RIGHT_DRIVE_MOTOR_NAME = "rear_right";
    final String SHOOTER_MOTOR_NAME = "motor_0";
    final String TRIGGER_SERVO_NAME = "servo_1";

    private final ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive mecanumDrive = null;
    private Shooter shooter = null;
    private GoBildaPinpointDriver odo;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, FRONT_LEFT_DRIVE_MOTOR_NAME);
        DcMotor frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_DRIVE_MOTOR_NAME);
        DcMotor rearLeft  = hardwareMap.get(DcMotor.class, REAR_LEFT_DRIVE_MOTOR_NAME);
        DcMotor rearRight = hardwareMap.get(DcMotor.class, REAR_RIGHT_DRIVE_MOTOR_NAME);

        DcMotor shooterMotor = hardwareMap.get(DcMotor.class, SHOOTER_MOTOR_NAME);
        Servo triggerServo = hardwareMap.get(Servo.class, TRIGGER_SERVO_NAME);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        mecanumDrive = new MecanumDrive(frontLeft, frontRight, rearLeft, rearRight);
        shooter = new Shooter(shooterMotor, triggerServo);

        telemetry.addData("Status", "Initialized");
        System.out.println("TeleOp_Starter: Initializing Logging"); // where does this go?
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double forward = -gamepad1.left_stick_y; // TODO: why is this negative?
        double strafe = gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        mecanumDrive.drive(forward, strafe, rotate);

        double shooterSpeed = 0.0;
        if (gamepad2.a) {
            shooterSpeed = 0.75;
        }
        shooter.spinUp(shooterSpeed);

        boolean trigger = gamepad2.rightBumperWasPressed();
        if (trigger) {
            shooter.shoot();
        }

        // Display Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive params", "forward (%.2f), strafe (%.2f), rotate (%.2f)", forward, strafe, rotate);
        telemetry.addData("Shooter params", "shooterSpeed (%.2f), trigger (%b)", shooterSpeed, trigger);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
