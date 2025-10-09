package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotor shooterMotor;
    private double ticksPerRev;

    public void init(HardwareMap hwMap){
        shooterMotor = hwMap.get(DcMotor.class,"shooter_motor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setShooterSpeed(double speed){
        shooterMotor.setPower(speed);
    }

    public double getTicksPerRev() {
        return shooterMotor.getCurrentPosition() / ticksPerRev;
    }
}
