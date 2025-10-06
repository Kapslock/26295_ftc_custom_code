

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorTester {

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
            return DetectedColor.GREEN;
        }
        else if (normRed > 0.4 && normBlue > 0.4 && normGreen < 0.3) {
            return DetectedColor.PURPLE;
        }
        return DetectedColor.UNKNOWN;



    }
}
