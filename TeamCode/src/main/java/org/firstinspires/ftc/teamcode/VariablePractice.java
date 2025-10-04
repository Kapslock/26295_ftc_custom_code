package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class VariablePractice extends OpMode {

    @Override
    public void init() {
        int teamNumber = 31547;
        double speedMotor = 0.5;
        boolean shooterOn = true;

        telemetry.addData("Team number", teamNumber);
        telemetry.addData("Motor speed", speedMotor);
        telemetry.addData("Shooter status", shooterOn);
    }

    @Override
    public void loop() {

    }
}
