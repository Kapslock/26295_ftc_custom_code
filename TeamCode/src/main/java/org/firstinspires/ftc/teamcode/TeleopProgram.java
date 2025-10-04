package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@TeleOp
public class TeleopProgram extends OpMode {
    DriveTrain driveTrain = new DriveTrain();
    Shooter shooter = new Shooter();
    Intake intake = new Intake();

    @Override
    public void init() {
        driveTrain.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y > 0) {
            driveTrain.setBackLeftMotorSpeed(0.5);
            driveTrain.setFrontLeftMotorSpeed(-0.5);
        } else if (gamepad1.left_stick_x < 0) {
            driveTrain.setBackLeftMotorSpeed(0.5);
            driveTrain.setFrontLeftMotorSpeed(-0.5);
        } else {
            driveTrain.setBackLeftMotorSpeed(0);
            driveTrain.setFrontLeftMotorSpeed(0);
        }

        if (gamepad1.right_stick_y > 0) {
            driveTrain.setBackRightMotorSpeed(0.5);
            driveTrain.setFrontRightMotorSpeed(-0.5);
        } else if (gamepad1.left_stick_x < 0) {
            driveTrain.setBackRightMotorSpeed(0.5);
            driveTrain.setFrontRightMotorSpeed(-0.5);
        } else {
            driveTrain.setBackRightMotorSpeed(0);
            driveTrain.setFrontRightMotorSpeed(0);
        }

        if (gamepad1.left_trigger > 0) {
            driveTrain.setBackRightMotorSpeed(0.5);
            driveTrain.setFrontRightMotorSpeed(-0.5);
            driveTrain.setBackLeftMotorSpeed(0.5);
            driveTrain.setFrontLeftMotorSpeed(-0.5);
        } else {
            driveTrain.setBackRightMotorSpeed(0);
            driveTrain.setFrontRightMotorSpeed(0);
            driveTrain.setBackLeftMotorSpeed(0);
            driveTrain.setFrontLeftMotorSpeed(0);
        }

        if (gamepad1.right_trigger > 0) {
            driveTrain.setBackRightMotorSpeed(-0.5);
            driveTrain.setFrontRightMotorSpeed(0.5);
            driveTrain.setBackLeftMotorSpeed(-0.5);
            driveTrain.setFrontLeftMotorSpeed(0.5);
        } else {
            driveTrain.setBackRightMotorSpeed(0);
            driveTrain.setFrontRightMotorSpeed(0);
            driveTrain.setBackLeftMotorSpeed(0);
            driveTrain.setFrontLeftMotorSpeed(0);
        }

        if (gamepad1.a) {
            shooter.setShooterSpeed(1);
        } else {
            shooter.setShooterSpeed(0);
        }

        if (gamepad1.b) {
            intake.setIntakeSpeed(1);
        } else {
            intake.setIntakeSpeed(0);
        }
    }
}
