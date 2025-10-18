package org.firstinspires.ftc.teamcode.BBcode.MechanismControllers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class OuttakeShooter {
    OpMode _opMode;
    DcMotorEx _launcher;
    public OuttakeShooter (OpMode opMode)
    {
        _opMode = opMode;
        _launcher = _opMode.hardwareMap.tryGet(DcMotorEx.class, "launcher");
    }
    //-----------------------------------------
    //Variable Storage:
    double shortShot = 0.6;
    double longShot = 0.8;
    double stopped = 0;

    //-----------------------------------------

    public void ShortShot() {
        LauncherCustom(shortShot);
    }
    public void LongShot() {
        LauncherCustom(longShot);
    }
    public void Stopped() {
        LauncherCustom(stopped);
    }
    public void LauncherCustom(double power)
    {
        if (_launcher == null)
        {
            _opMode.telemetry.addLine("Transfer Motor not found!");
        } else {
            _launcher.setPower(power);
        }
    }
}
