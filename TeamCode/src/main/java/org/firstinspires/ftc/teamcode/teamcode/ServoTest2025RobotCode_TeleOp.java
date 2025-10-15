package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "ServoTest2025RobotCode_TeleOp", group = "Robot")
public class ServoTest2025RobotCode_TeleOp extends OpMode {

    NormalizedColorSensor colorSensor0;
    double hue;
    Servo carousel;
    Servo kicker;

    @Override
    public void init() {
        carousel = hardwareMap.get(Servo.class, "carousel");
        kicker = hardwareMap.get(Servo.class, "kicker");
        colorSensor0 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor0");
        telemetry.addData("Status", "Initialized says Teammate");
        telemetry.update();
    }

    @Override
    public void loop() {
        testColorSensors();

        if (gamepad1.x) {
            carousel.setPosition(0.5);
        }
        if (gamepad1.y) {
            carousel.setPosition(0.0);
        }

        if (gamepad1.a) {
            kicker.setPosition(0.5);
        }

        kicker.setPosition(0.0);
    }

    public void testColorSensors() {
        telemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSensor0).getLightDetected());
        NormalizedRGBA colors = colorSensor0.getNormalizedColors();
        hue = JavaUtil.colorToHue(colors.toColor()); // <------ New code

        //Determining the amount of red, green, and blue
        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);

        //Determining HSV and alpha
        telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
        telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors.toColor()));
        telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors.toColor()));
        telemetry.addData("Alpha", "%.3f", colors.alpha);
        telemetry.update();
    }
}


