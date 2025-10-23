package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DriveLaunchMode", group = "OpModes")
public class DriveLaunchMode extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    private final ElapsedTime matchTime = new ElapsedTime();
    private final double[] powerSteps = {0.1, 0.67, 0.72};
    LaunchSystem launchSystem = new LaunchSystem();
    double startWait = 0.0;
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;

    @Override
    public void init() {
        //Initialize hardware
        drive.init(hardwareMap, telemetry);

        //NOTE: The MIN is for the UPPER limit, the MAX if the the LOWER limit
        launchSystem.init(0.10, 0.24, powerSteps, hardwareMap, telemetry);
    }

    @Override
    public void start() {
        matchTime.reset();
    }

    @Override
    public void loop() {
        //Take controller inputs
        double forward = -1 * gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x * 1.1;
        double slow = 1.0;

        if (gamepad1.left_trigger > 0.4) {
            slow = 0.5;
        } else if (gamepad1.right_trigger > 0.4) {
            slow = 2;
        }

        if (gamepad1.dpad_up && !lastDpadUp) {
            launchSystem.stepUpPower();
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            launchSystem.stepDownPower();
        }

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        if (gamepad1.triangleWasPressed()) {
            launchSystem.toggleLauncher();
        }

        if (gamepad1.squareWasPressed()) {
            launchSystem.toggleIntake();
        }


        if (gamepad1.crossWasPressed()) {
            launchSystem.liftUp();
            startWait = matchTime.milliseconds();
        }

        if (matchTime.milliseconds() >= startWait + 100) {
            launchSystem.liftDown();
        }


        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);
        telemetry.addData("speed", slow);
        launchSystem.updateTelemetry(telemetry);

        drive.driveFieldOriented(forward, strafe, rotate, slow, telemetry);
    }
}
