package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DriveOpMode", group = "OpModes")
public class DriveOpMode extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate, slow;

    @Override
    public void init() {

        //Initializes hardware
        drive.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        //Takes controller inputs
        forward = -1 * gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x * 1.1;

        //Alter driving speed (Left trigger = half speed, right trigger = double Speed)
        if (gamepad1.left_trigger > 0.4) {
            slow = 0.5;
        } else if (gamepad1.right_trigger > 0.4) {
            slow = 2;
        } else{
            slow = 1;
        }

        //Recenter field-centric driving
        //-STAY STILL FOR AT LEAST 0.25 SECONDS WHILE DOING SO FOR ACCURACY-
        if (gamepad1.dpad_up) {
            drive.OdoReset(telemetry);
        }

        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);
        telemetry.addData("speed", slow);

        /*
        Use this for non field-centric code:
        drive.drive(forward, strafe, rotate, slow);
         */
        drive.driveFieldOriented(forward, strafe, rotate, slow, telemetry);

    }
}
