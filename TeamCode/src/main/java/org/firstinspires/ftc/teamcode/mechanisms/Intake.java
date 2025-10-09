package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor;
    private CRServo ballTriggerServo;

    public void init(HardwareMap hwMap){
        intakeMotor = hwMap.get(DcMotor.class,"intake_motor");
        ballTriggerServo = hwMap.get(CRServo.class, "ball_triger");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballTriggerServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.setPower(speed);
    }

    public void ballServoActivation(double power) {
        ballTriggerServo.setPower(power);
    }
}
