package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    private DcMotor backRightMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontLeftMotor;

    public void init(HardwareMap hwMap) {
        backRightMotor = hwMap.get(DcMotor.class, "back_right_motor");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");
        frontLeftMotor = hwMap.get(DcMotor.class, "front_left_motor");

        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setBackRightMotorSpeed(double speed) {
        backRightMotor.setPower(speed);
    }
    public void setFrontRightMotorSpeed(double speed) {
        frontRightMotor.setPower(speed);
    }
    public void setBackLeftMotorSpeed(double speed) {
        backLeftMotor.setPower(speed);
    }
    public void setFrontLeftMotorSpeed(double speed) {
        frontLeftMotor.setPower(speed);
    }
}
