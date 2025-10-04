package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBench {
    private DigitalChannel touchSensor;

    public void init(HardwareMap haMap) {
        touchSensor = haMap.get(DigitalChannel.class,"touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isTouchSensorActive(){
        return !touchSensor.getState();
    }

    public boolean isTouchSensorRealeased(){
        return touchSensor.getState();
    }
}
