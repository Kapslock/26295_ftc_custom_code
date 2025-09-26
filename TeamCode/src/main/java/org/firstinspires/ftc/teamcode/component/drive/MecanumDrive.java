package org.firstinspires.ftc.teamcode.component.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrive {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    public MecanumDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor rearLeft, DcMotor rearRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
    }

    public void drive(double forward, double strafe, double rotate) {

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        frontLeft.setPower((forward + strafe + rotate) / denominator);
        rearLeft.setPower((forward - strafe + rotate) / denominator);
        frontRight.setPower((forward - strafe - rotate) / denominator);
        rearRight.setPower((forward + strafe - rotate) / denominator);
    }










}
