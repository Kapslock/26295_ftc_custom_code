package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOp_ShooterPrototype", group="Test")
public class TeleOp_ShooterPrototype extends OpMode {
    final String SHOOTER_MOTOR_NAME = "shooter";
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor shooter;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        shooter  = hardwareMap.get(DcMotor.class,SHOOTER_MOTOR_NAME);

        shooter.setDirection(DcMotor.Direction.FORWARD);

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
        boolean runShooter =  gamepad1.right_bumper;
        double shooterPower = 0.6;

        if (runShooter) {
            shooter.setPower(shooterPower);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData(
            "Shooter", "activated (%b), power (%f)",
            runShooter,
            runShooter ? shooterPower : 0.0f
        );
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
