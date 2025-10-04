package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor;

    public void init(HardwareMap hwMap){
        intakeMotor = hwMap.get(DcMotor.class,"intake_motor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.setPower(speed);
    }

}
