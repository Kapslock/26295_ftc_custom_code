package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "HelloWorld", group = "Tutorials")
public class HelloWorld extends OpMode {

    @Override
    public void init() {
        telemetry.addData("Hello,", "World2222");
    }

    @Override
    public void loop() {

    }
}
