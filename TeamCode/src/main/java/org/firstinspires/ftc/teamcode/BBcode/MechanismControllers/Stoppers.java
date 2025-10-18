package org.firstinspires.ftc.teamcode.BBcode.MechanismControllers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Stoppers {
    OpMode _opMode;
    Servo _rightStopper;
    Servo _leftStopper;
    public Stoppers (OpMode opMode)
    {
        _opMode = opMode;
        _rightStopper = _opMode.hardwareMap.tryGet(Servo.class, "rightStopper");
        _leftStopper = _opMode.hardwareMap.tryGet(Servo.class, "leftStopper");
    }
    //-----------------------------------------
    //Variable Storage:
    double transferUp = 0.75;

    //-----------------------------------------

    public void tranferUpForShot() {
        rightStopperCustom(transferUp);
        leftStopperCustom(transferUp);
    }
    public void rightStopperCustom(double time)
    {
        if (_rightStopper == null)
        {
            _opMode.telemetry.addLine("rightStopper Servo not found!");
        } else {
            _rightStopper.setPosition(time);
        }
    }
    public void leftStopperCustom(double time)
    {
        if (_leftStopper == null)
        {
            _opMode.telemetry.addLine("leftStopper Servo not found!");
        } else {
            _leftStopper.setPosition(time);
        }
    }
}
