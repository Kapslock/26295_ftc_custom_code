package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name = "Mechanisms")
public class Mechanisms {
    private static final double sortingMotor_TPR = 384.5; // Encoder ticks per revolution
    private static final int sorter_pos1 = 67; // Encoder ticks per revolution
    private static final int sorter_pos2 = 67; // Encoder ticks per revolution
    private static final int sorter_pos3 = 67; // Encoder ticks per revolution
    private static final double intakeTargetVelocity = 363;


    public DcMotorEx intakeMotor;
    public DcMotor sortingMotor;

    public void initIntakeSystem(HardwareMap hardwareMap) {
        sortingMotor = hardwareMap.get(DcMotor.class, "sortingMotor"); // 435 rpm dc motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor"); // 1150 rpm dc motor

        sortingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sortingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sortingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sortingMotor.setPower(0);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setPower(0);
    }

    public void engageIntake(HardwareMap hardwareMap) {
        // intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setVelocity(intakeTargetVelocity);
    }

    public void disengageIntake(HardwareMap hardwareMap) {
        // intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setVelocity(0);
    }


    public void indexArtifacts() {
        sortingMotor.setTargetPosition(sorter_pos1);
        sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortingMotor.setPower(0.5);
        // record color
        sortingMotor.setTargetPosition(sorter_pos2);
        sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortingMotor.setPower(0.5);
        // record color
        sortingMotor.setTargetPosition(sorter_pos3);
        sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortingMotor.setPower(0.5);
    }

    public void cycleArtifacts(int position) {
        if (position == 0) {
            sortingMotor.setTargetPosition(sorter_pos1);
            sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sortingMotor.setPower(0.5);
        } else if (position == 1) {
            sortingMotor.setTargetPosition(sorter_pos1);
            sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sortingMotor.setPower(0.5);
        } else if (position == 2) {
            sortingMotor.setTargetPosition(sorter_pos1);
            sortingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sortingMotor.setPower(0.5);
        } else {
            telemetry.addData("Status:", "in cycleArtifacts, invalid input");
        }

    }
}



