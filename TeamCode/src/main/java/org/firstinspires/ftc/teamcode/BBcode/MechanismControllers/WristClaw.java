package org.firstinspires.ftc.teamcode.BBcode.MechanismControllers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class WristClaw {
    OpMode _opMode;
    Servo _claw;
    ChristmasLight _light;
    public WristClaw (OpMode opMode)
    {
        _opMode = opMode;
        _claw = _opMode.hardwareMap.tryGet(Servo.class, "claw");
        _light = new ChristmasLight(opMode);
    }
    //-----------------------------------------
    //Variable Storage:
    double openPosition = 0.73;

    //-----------------------------------------

    public void OpenClaw() {
        ClawCustom(openPosition);
        _light.blue();
    }
    public void ClawCustom(double position)
    {
        if (_claw == null)
        {
            _opMode.telemetry.addLine("Claw Servo not found!");
        } else {
            _claw.setPosition(position);
        }
    }
}
