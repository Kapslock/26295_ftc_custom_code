package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MecanumDriveFieldOriented extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate, slow;

    @Override
    public void init() {
        drive.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        if (gamepad1.left_trigger > 0.4) {
            slow = 0.5;
        } else{
            slow = 1;
        }

        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);
        telemetry.addData("speed", slow);

        drive.driveFieldOriented(forward, strafe, rotate, slow, telemetry);

    }
}
