package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeTesting {
    public DcMotor intake;

    public void spin() {

        intake.setDirection(DcMotor.Direction.FORWARD);
    }
}
