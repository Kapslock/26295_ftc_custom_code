package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.TestBench;

import java.util.HashMap;

@Disabled
public class TouchSensorPractice extends OpMode {
    TestBench TestBench = new TestBench();

    @Override
    public void init() {
        TestBench.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Touch sensor pressed", TestBench.isTouchSensorRealeased());
    }
}
