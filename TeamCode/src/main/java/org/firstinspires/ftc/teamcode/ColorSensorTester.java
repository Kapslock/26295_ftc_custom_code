

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "LinearActuatorTester")
public class ColorSensorTester extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        NormalizedColorSensor colorSensor;
        /*enum DetectedColor {

            RED,
            BLUE,
            YELLOW,
            PURPLE,
            GREEN,
            UNKNOWN

        }*/
        waitForStart();

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_distance");

        while (opModeIsActive()) {

            /*public DetectedColor getDetectedColor(Telemetry telemetry){
                NormalizedRGBA colors = colorSensor.getNormalizedColors(); //returns 4 values

                float normRed, normGreen, normBlue;
                normRed = colors.red / colors.alpha;
                normGreen = colors.green / colors.alpha;
                normBlue = colors.blue / colors.alpha;

                telemetry.addData("red", normRed);
                telemetry.addData("green", normGreen);
                telemetry.addData("blue", normBlue);

                if (normGreen > 0.5 && normRed < 0.3 && normBlue < 0.3) {
                    telemetry.addData("Color detected", "green");
                    return DetectedColor.GREEN;
                } else if (normRed > 0.4 && normBlue > 0.4 && normGreen < 0.3) {
                    telemetry.addData("Color detected", "purple");
                    return DetectedColor.PURPLE;
                }
                return DetectedColor.UNKNOWN;
            }*/
        }
    }

    /*

    NormalizedColorSensor colorSensor;
    public enum DetectedColor {

        RED,
        BLUE,
        YELLOW,
        PURPLE,
        GREEN,
        UNKNOWN

    }

    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_distance");
    }


    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors= colorSensor.getNormalizedColors(); //returns 4 values

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        if(normGreen > 0.5 && normRed < 0.3 && normBlue < 0.3) {
            telemetry.addData("Color detected","green");
            return DetectedColor.GREEN;
        }
        else if (normRed > 0.4 && normBlue > 0.4 && normGreen < 0.3) {
            telemetry.addData("Color detected","purple");
            return DetectedColor.PURPLE;
        }
        return DetectedColor.UNKNOWN;



    }
    */

}
