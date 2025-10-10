package org.firstinspires.ftc.teamcode;

import static android.icu.lang.UProperty.MATH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TeleopGamePadStuff")
public class TeleopMain extends LinearOpMode{


    //private static final double TICKS_PER_REVOLUTION = 1538.0;
    private ElapsedTime runtime = new ElapsedTime();


    public TeleopMain() {

    }

   // @Override
    public void runOpMode() throws InterruptedException {
        TeleopDrivetrain drivetrain = new TeleopDrivetrain(this);
        Mechanisms mech = new Mechanisms();
        mech.initTelemetry(telemetry);

        drivetrain.initDriveTrain((hardwareMap));
        mech.initOuttakeSystem(hardwareMap);
      //  mech.initMechanisms(hardwareMap);

        //Checks what team color we are

        telemetry.addData("Status","READY TO NUT");

        this.waitForStart();

        //This is loop that checks the gamepad for inputs every iteration
        while (opModeIsActive()) {
           // mech.updateZeroPosition();

           double time = runtime.startTime();
//            telemetry.addData("RunTime", time);
//            telemetry.addData("ifBlue",mech.ifBlueTeam);
//            telemetry.addData("ifRed", mech.ifRedTeam);

            //drives the robot
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            //double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower = (y + x + rx)/1.5;
            double backLeftPower = (y - x + rx)/1.5;
            double frontRightPower = (y - x - rx)/1.5;
            double backRightPower = (y + x - rx)/1.5;

            drivetrain.frontLeft.setPower(frontLeftPower);
            drivetrain.frontRight.setPower(frontRightPower);
            drivetrain.backLeft.setPower(backLeftPower);
            drivetrain.backRight.setPower(backRightPower);



//Find out what to put in targetInSeconds
            if(gamepad1.left_stick_y < 0.0) {
                drivetrain.moveForward(Math.abs(gamepad1.left_stick_y));
            } else if (gamepad1.left_stick_y > 0.0) {
                drivetrain.moveBackwards(Math.abs(gamepad1.left_stick_y));
            }

            if(gamepad1.left_stick_x < 0.0){
                drivetrain.strafeLeft(Math.abs(gamepad1.left_stick_x));
            } else if (gamepad1.left_stick_x > 0.0) {
                drivetrain.strafeRight(Math.abs(gamepad1.left_stick_x));
            }

        if (gamepad2.dpad_left){
              //  mech.simplePivotLimit1();
                //mech.extendViperSlide("backward");

            }
            if (gamepad2.dpad_right){
              //  mech.simplePivotLimit1();
               // mech.extendViperSlide("forward");
            }

            if(gamepad2.dpad_down){
                //mech.pivotLimit1();
             //   mech.armMotorPivot("down");
            }
            if(gamepad2.dpad_up){
                //mech.pivotLimit1();
               // mech.armMotorPivot("up");
            }

            if (gamepad2.a) {
              //  mech.SpecimenScoringPosition();
            }
            //block pickup positions on wall
            if (gamepad2.b) {
            //    mech.SpecimenPickupPosition();
            }
            //block pickup position from floor
            if(gamepad2.y){
              //  mech.BlockPickupPosition();
            }
            if (gamepad2.x) {
                //mech.SpecimenViperPosition();
            }

            if(gamepad2.right_bumper){
                //mech.setClawPivot("up");

            }
            if(gamepad2.left_bumper) {
                //mech.setClawPivot("down");

            }

            if(gamepad2.right_trigger > 0.1){
                mech.outtakeMotorStart(gamepad2.right_trigger);
            } else {
                mech.outtakeMotorStop();
            }
            if(gamepad2.left_trigger > 0.1) {
                //mech.setClawPivot("down");

            }


            telemetry.update();
            //sleep(100);
        }
    }



}

