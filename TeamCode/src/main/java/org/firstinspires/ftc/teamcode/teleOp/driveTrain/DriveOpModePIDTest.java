package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriveOpModePIDTest extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate, slow;
    double pidP = 0.1;
    double pidI = 0.0;
    double pidD = 0.0;


    @Override
    public void init() {

        drive.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        forward = -1 * gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        if (gamepad1.left_trigger > 0.4) {
            slow = 0.5;
        } else if (gamepad1.right_trigger > 0.4) {
            slow = 2;
        } else{
            slow = 1;
        }

        if (gamepad1.dpad_up) {
            drive.OdoReset(telemetry);
        }
        if (gamepad1.triangle) {
            pidP += 0.1;
        }
        if (gamepad1.circle) {
            pidI += 0.01;
        }
        if (gamepad1.cross) {
            pidD += 0.01;
        }

        telemetry.addData("Elapsed Time", getRuntime());
        telemetry.addLine();

        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);
        telemetry.addData("speed", slow);

        drive.turnToHeading(rotate, slow, telemetry, pidP, pidI, pidD);

        drive.driveFieldOriented(forward, strafe, rotate, slow, telemetry);
        //drive.drive(forward, strafe, rotate, slow, telemetry);

    }
}
