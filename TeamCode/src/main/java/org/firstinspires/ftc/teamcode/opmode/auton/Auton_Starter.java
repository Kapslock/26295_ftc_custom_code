package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.component.drive.TankDrive;

@Autonomous(name="Auton_TankStarter", group="Iterative OpMode")
public class Auton_Starter extends OpMode
{
    final String LEFT_DRIVE_MOTOR_NAME = "left_drive";
    final String RIGHT_DRIVE_MOTOR_NAME = "right_drive";
    private final ElapsedTime runtime = new ElapsedTime();
    private TankDrive tankDrive = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        DcMotor leftDrive  = hardwareMap.get(DcMotor.class, LEFT_DRIVE_MOTOR_NAME);
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, RIGHT_DRIVE_MOTOR_NAME);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        tankDrive = new TankDrive(leftDrive, rightDrive);

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

        double leftPower = 1.0f;
        double rightPower = 1.0f;
        double scaleFactor = 0.1;

        // run forward for 1 second then stop
        if (runtime.seconds() > 1.0f) {
            leftPower = 0.0f;
            rightPower = 0.0f;
        }
        // Invoke the TankDrive to control the motors
        tankDrive.drive(leftPower, rightPower, scaleFactor);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
