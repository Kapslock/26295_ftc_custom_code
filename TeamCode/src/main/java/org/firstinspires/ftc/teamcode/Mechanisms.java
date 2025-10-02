package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Mechanisms")
public class Mechanisms {
   public DcMotor intakeMotor;

   public void takeInBall(HardwareMap hardwareMap) {
      intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
      intakeMotor.setPower();

   }
}
