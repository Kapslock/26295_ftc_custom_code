package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleopGamePadStuff")
public class TeleopGamePadStuff {


    //private static final double TICKS_PER_REVOLUTION = 1538.0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        TeleopDrivetrain drivetrain = new TeleopDrivetrain(this);
       // Mechanisms mech = new Mechanisms(this);

        drivetrain.initDriveTrain((hardwareMap));
      //  mech.initMechanisms(hardwareMap);

        //Checks what team color we are

        telemetry.addData("Status","READY TO NUT");





        waitForStart();

//        boolean press = false;
//        boolean pressStart = false;
//
//        while(!press){
//            telemetry.addLine("Choose Team");
//            telemetry.addData("Red Team:", "Player 1, press UP");
//            telemetry.addData("Blue Team: ", "Player 1, press DOWN");
//            telemetry.update();
//            if(gamepad1.dpad_up){
//                mech.ifRedTeam = true;
//                mech.ifBlueTeam = false;
//                telemetry.addData("Team Selected", "RED");
//                press = true;
//
//            } else if (gamepad1.dpad_down){
//                mech.ifBlueTeam = true;
//                mech.ifRedTeam = false;
//                telemetry.addData("Team Selected", "BLUE");
//                press = true;
//
//            }
//            telemetry.update();
//        }
//
//        while(!pressStart) {
//            telemetry.addData("Ready to Start?", "Press START");
//            if (gamepad1.start){
//                pressStart = true;
//            }
//        }



        if (isStopRequested()) {
            return;
        }

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
                drivetrain.moveForward( gamepad1.left_stick_y, 5);
            } else if (gamepad1.left_stick_y > 0.0) {
                drivetrain.moveBackwards(gamepad1.left_stick_y,5);
            }

            if(gamepad1.left_stick_x < 0.0){
                drivetrain.strafeLeft(gamepad1.left_stick_x, 5);
            } else if (gamepad1.left_stick_x > 0.0) {
                drivetrain.strafeRight(gamepad1.left_stick_x, 5);
            }


            //gamepad1
            //Position to score specimen
     /*       if(gamepad1.) {
                mech.SpecimenScoringPositionV3();
            }
            //Position to score specimen in the basket
            if (gamepad1.x) {
                mech.BasketScorePosition();
            }
*/

            //controls gamepad2 claw open and close
          //  mech.openClaw(gamepad2.left_trigger);
            //mech.closeClaw(gamepad2.right_trigger);

            //extends the viper slide
            //gamepad 2 functions
          //  telemetry.addData("ViperSlide", mech.viperSlide.getCurrentPosition());
            if (gamepad2.dpad_left){
              //  mech.simplePivotLimit1();
                //mech.extendViperSlide("backward");

            }
            if (gamepad2.dpad_right){
              //  mech.simplePivotLimit1();
               // mech.extendViperSlide("forward");
            }

            //pivots the arm
           // telemetry.addData("viperPivot", mech.viperPivot.getCurrentPosition());
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



            //pivots the claw
            telemetry.addData("ClawPivotPosition",mech.pivot.getPosition());
            if(gamepad2.right_bumper){
                //mech.setClawPivot("up");

            }
            if(gamepad2.left_bumper) {
                //mech.setClawPivot("down");

            }


            telemetry.update();
            //sleep(100);
        }
    }

    private void waitForStart() {
    }
}

