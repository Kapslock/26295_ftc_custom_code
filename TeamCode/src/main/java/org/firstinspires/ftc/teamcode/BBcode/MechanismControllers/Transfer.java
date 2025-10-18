package org.firstinspires.ftc.teamcode.BBcode.MechanismControllers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Transfer {
    OpMode _opMode;
    DcMotorEx _tranfer;
    public Transfer (OpMode opMode)
    {
        _opMode = opMode;
        _tranfer = _opMode.hardwareMap.tryGet(DcMotorEx.class, "tranfer");
    }
    //-----------------------------------------
    //Variable Storage:
    double transfering = 0.75;
    double stopped = 0;

    //-----------------------------------------

    public void tranferingToStoppers() {
        TransferCustom(transfering);
    }
    public void stopped() {
        TransferCustom(stopped);
    }
    public void TransferCustom(double power)
    {
        if (_tranfer == null)
        {
            _opMode.telemetry.addLine("Transfer Motor not found!");
        } else {
            _tranfer.setPower(power);
        }
    }
}
