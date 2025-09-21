package org.firstinspires.ftc.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "RB Auto")
public class RB_Auto {
  
  private DcMotor frontLeftMotor;
  private DcMotor backLeftMotor;
  private DcMotor frontRightMotor;
  private DcMotor backRightMotor;

  @Override
  public void runOpMode(){

    frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
    backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
    servoRotate = hardwareMap.get(CRServo.class, "servoRotate");

    waitForStart();
    
  }
  
}

