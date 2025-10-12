package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest2025RobotCode_TeleOp", group = "Robot")
public class ServoTest2025RobotCode_TeleOp extends OpMode {

    Servo carousel;
    Servo kicker;

    @Override
    public void init() {
        carousel = hardwareMap.get(Servo.class, "carousel");
        kicker = hardwareMap.get(Servo.class, "kicker");
    }

    @Override
    public void loop() {
        if(gamepad1.x) {
            carousel.setPosition(0.5);
        }
        if (gamepad1.y) {
            carousel.setPosition(0.0);
        }

        if(gamepad1.a) {
            kicker.setPosition(0.5);
        }

        kicker.setPosition(0.0);
    }
}
