package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@TeleOp
public class TeleopProgram extends OpMode {
    Shooter shooter = new Shooter();
    Intake intake = new Intake();
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate;

    @Override
    public void init() {
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        drive.driveFieldRelative(forward, strafe, rotate);

        if (gamepad1.b) {
            intake.ballServoActivation(1);
        } else {
            intake.ballServoActivation(0);
        }

        if (gamepad1.right_trigger > 0) {
            shooter.setShooterSpeed(1);
        } else {
            shooter.setShooterSpeed(0);
        }

        if (gamepad1.left_trigger > 0) {
            intake.setIntakeSpeed(1);
        } else {
            intake.setIntakeSpeed(0);
        }

        if (gamepad1.a) {
            shooter.setShooterSpeed(1);
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            intake.setIntakeSpeed(1);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            for (int i = 0;i<2;i++) {
                intake.ballServoActivation(1);
                try {
                    Thread.sleep(1250);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                intake.ballServoActivation(0);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
            intake.setIntakeSpeed(0);
            shooter.setShooterSpeed(0);
        }
    }
}
