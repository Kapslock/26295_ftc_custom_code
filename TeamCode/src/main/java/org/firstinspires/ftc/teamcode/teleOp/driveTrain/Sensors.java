package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.hardware.HardwareBuffer;
import android.hardware.Sensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Sensors {
    private static final Logger log = LoggerFactory.getLogger(Sensors.class);

    private RevColorSensorV3 color = (RevColorSensorV3) hardwareMap.colorSensor.get("colorV3");

    public void init (HardwareMap hwMap) {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Sensor Name", color.getDeviceName());
        telemetry.addLine();
        telemetry.addData("Red", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue", color.blue());

        telemetry.update();

    }
}
