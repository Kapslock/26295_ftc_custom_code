package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
-IF YOU ARE A NEW CODER START HERE-

This is old Code from last 2024's Into the Deep competition.

It was our main code and just reading through it will teach you a lot about how to
program a simple LinearOpMode.

Most of the code was written by Kylah, a former member.

This is just a learning resource, please do not edit the code whatsoever.
*/

@Disabled
@TeleOp(name="Controller24", group="Linear OpMode")
public class Controller24 extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor vertarmLeft = null;
    private DcMotor vertarmRight = null;

    private DcMotor horarm = null;
    //private IMU imu = null;



    private Servo clawRotate = null;
    private Servo clawUpdown = null;
    private Servo clawGrabber = null;
    private Servo armGrabber = null;
    private Servo armRotate = null;//big
    private Servo armTurn = null;//little






    @Override
    public void runOpMode() {

        double speed = 0.5;//drive speed
        int vertarmPlace=0;//vertical arm start lowest
        //boolean ifArmFront=true;
        int armPosition=0;



        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor  = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");//0
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");

        horarm  = hardwareMap.get(DcMotor.class, "horarm");
        vertarmLeft  = hardwareMap.get(DcMotor.class, "vertarmLeft");
        vertarmRight= hardwareMap.get(DcMotor.class, "vertarmRight");

        //imu = hardwareMap.get(IMU.class, "imu");

        clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        clawGrabber = hardwareMap.get(Servo.class, "clawGrabber");
        armGrabber = hardwareMap.get(Servo.class, "armGrabber");
        armRotate = hardwareMap.get(Servo.class, "armRotate");
        clawUpdown = hardwareMap.get(Servo.class, "clawUpdown");
        armTurn = hardwareMap.get(Servo.class, "armTurn");








        // ########################################################################################
        // !!!!           IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        vertarmLeft.setDirection(DcMotor.Direction.FORWARD);
        vertarmRight.setDirection(DcMotor.Direction.FORWARD);

        horarm.setDirection(DcMotor.Direction.FORWARD);

        clawRotate.setDirection(Servo.Direction.FORWARD);
        clawGrabber.setDirection(Servo.Direction.FORWARD);
        armGrabber.setDirection(Servo.Direction.FORWARD);
        armRotate.setDirection(Servo.Direction.FORWARD);
        clawUpdown.setDirection(Servo.Direction.FORWARD);
        armTurn.setDirection(Servo.Direction.FORWARD);




        horarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertarmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertarmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vertarmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertarmRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        horarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            //prepare:
            if (this.gamepad2.dpad_right) {
                clawRotate.setPosition(0.65);//mid
                clawGrabber.setPosition(0.57);//open
                armGrabber.setPosition(0.32);//open
                armRotate.setPosition(.37);//front
                armTurn.setPosition(0.78);//facing down
                clawUpdown.setPosition(0.78);//down
                armPosition=0;
            }




            //user 1:


            //Change Speed
            if (this.gamepad1.right_trigger>0.2) {
                speed = .5;//slow
            }
            else {
                speed = 1;//normal
            }



            //calc speed
            double max;//the max value of the four wheels

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > speed) {
                leftFrontPower /= max / speed;
                rightFrontPower /= max / speed;
                leftBackPower /= max / speed;
                rightBackPower /= max / speed;
            }
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);


            //horizontal arm
            if (this.gamepad1.y && horarm.getCurrentPosition() < 780) {
                horarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                horarm.setPower(0.58);//extend(out)
            }
            else if (this.gamepad1.a) {
                horarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                horarm.setPower(-0.58);//retract(in)
            }
            else{
                horarm.setPower(0);
            }


            //claw up down 180
            if (this.gamepad1.dpad_down) {
                clawUpdown.setPosition(0.1);//turn up
                //clawUpdown.setPosition(clawUpdown.getPosition()+0.002);
            }
            else if (this.gamepad1.dpad_up) {
                clawUpdown.setPosition(0.78);//turn down
                //clawUpdown.setPosition(clawUpdown.getPosition()-0.002);
            }


            //Auto Submersible
            if (this.gamepad1.dpad_right){
                clawUpdown.setPosition(0.1);//turn up
                sleep(500);

                horarm.setTargetPosition(585);
                horarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horarm.setPower(0.8);//extend(out)

                clawRotate.setPosition(0.65);//claw mid

                clawGrabber.setPosition(0.57);//open

                while (horarm.isBusy()){
                }

                clawUpdown.setPosition(0.78);//turn down
            }
            else if (this.gamepad1.dpad_left){
                clawUpdown.setPosition(0.1);//turn up
                sleep(500);

                horarm.setTargetPosition(0);
                horarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horarm.setPower(-0.8);//extend(out)

                clawRotate.setPosition(0.65);//claw mid

                clawGrabber.setPosition(0.78);//close

//                while (horarm.isBusy()){
//                }
            }


            //clawRotate
            if (this.gamepad1.right_bumper && clawRotate.getPosition()>0.004) {
                //clawRotate.setPosition(0);
                clawRotate.setPosition(clawRotate.getPosition()-0.005);//turn clockwise/right
            }
            else if (this.gamepad1.left_bumper && clawRotate.getPosition()<0.996) {
                //clawRotate.setPosition(1);
                clawRotate.setPosition(clawRotate.getPosition()+0.005);//turn anticlockwise/left
            }


            //claw grabber
            if (this.gamepad1.b) {
                clawGrabber.setPosition(0.57);//open
            }
            else if (this.gamepad1.x) {
                clawGrabber.setPosition(0.78);//close
            }









            //user2
////           //updown
//            if (this.gamepad2.dpad_up && !vertarmRight.isBusy() && (vertarmPlace==0)) {
////                vertarmLeft.setTargetPosition(vertarmLeft.getCurrentPosition()-300);
//                vertarmRight.setTargetPosition(vertarmRight.getCurrentPosition()-300);
//
////                vertarmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);//up to 1st basket
//                vertarmRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);//up to 1st basket
//
////                vertarmLeft.setPower(-0.8);
//                vertarmRight.setPower(0.8);
//
//
////                vertarmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                //vertarmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                vertarmPlace=1;
//
////                if (!vertarmRight.isBusy()){
////                    vertarmPlace=1;
////                }
//            }
//            if (this.gamepad2.dpad_up && !vertarmRight.isBusy() && (vertarmPlace==1)) {
////                vertarmLeft.setTargetPosition(vertarmLeft.getCurrentPosition()-600);
//                vertarmRight.setTargetPosition(vertarmRight.getCurrentPosition()-600);
//
////                vertarmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);//up to 2nd basket
//                vertarmRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);//up to 2nd basket
//
////                vertarmLeft.setPower(-1);
//                vertarmRight.setPower(1);
//
//                vertarmPlace=2;
//            }
//            if (this.gamepad2.dpad_down && !vertarmRight.isBusy() && (vertarmPlace==2)) {
////                vertarmLeft.setTargetPosition(vertarmLeft.getCurrentPosition()+600);
//                vertarmRight.setTargetPosition(vertarmLeft.getCurrentPosition()+600);
//
////                vertarmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);//down back to initial form highest 2nd basket
//                vertarmRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);//down back to initial form highest 2nd basket
//
////                vertarmLeft.setPower(.8);
//                vertarmRight.setPower(-.5);
//
//                vertarmPlace=1;
//            }
//            if (this.gamepad2.dpad_down && !vertarmRight.isBusy() && (vertarmPlace==1)) {
////                vertarmLeft.setTargetPosition(vertarmLeft.getCurrentPosition()+300);
//                vertarmRight.setTargetPosition(vertarmLeft.getCurrentPosition()+300);
//
////                vertarmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);//down back to initial form 1st basket
//                vertarmRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);//down back to initial form 1st basket
//
////                vertarmLeft.setPower(.8);
//                vertarmRight.setPower(-.5);
//
//                vertarmPlace=0;
//            }
////            vertarmLeft.setTargetPosition(vertarmLeft.getCurrentPosition());
////            vertarmRight.setTargetPosition(vertarmRight.getCurrentPosition());
            if (this.gamepad2.right_trigger>0.2&&vertarmRight.getCurrentPosition()>-1300){
                vertarmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vertarmRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(this.gamepad2.dpad_left&&vertarmRight.getCurrentPosition()<-200){
                    vertarmLeft.setPower(0);
                    vertarmRight.setPower(0);
                }
                else{
                    vertarmLeft.setPower(-.8);//up
                    vertarmRight.setPower(-.8);
                }
            }
            else if(this.gamepad2.left_trigger>0.2){
                vertarmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vertarmRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                vertarmLeft.setPower(.4);//down
                vertarmRight.setPower(.4);//down

            }
            else{
                vertarmLeft.setPower(-0.05);
                vertarmRight.setPower(-0.05);
            }


//            if(this.gamepad2.dpad_down){
//                vertarmLeft.setPower(1);
//                vertarmRight.setPower(1);
//            }


            //grab auto
            if(this.gamepad2.dpad_up){
                armGrabber.setPosition(0.32);//arm open
                armTurn.setPosition(0.78);//face down
                clawRotate.setPosition(0.65);//claw mid
                clawGrabber.setPosition(0.79);//claw close
                sleep(300);
                clawUpdown.setPosition(0.1);//claw turn up
                armRotate.setPosition(0.37);//front/down
                sleep(1000);
                armGrabber.setPosition(0.01);//arm close
                sleep(450);
                clawGrabber.setPosition(0.57);//claw open
                sleep(450);
                armRotate.setPosition(0.74);//turn back
                clawUpdown.setPosition(0.78);//turn down
                armPosition=1;
            }


            //arm grabber
            if (this.gamepad2.a) {
                armGrabber.setPosition(0.32);//open
            }
            if (this.gamepad2.y) {
                armGrabber.setPosition(0.01);//close
            }


            //arm rotate
            /*if (this.gamepad2.b && ifArmFront) {
                armRotate.setPosition(0.6);//back/up8
                armTurn.setPosition(1);
                ifArmFront=false;
            }
            else if (this.gamepad2.b && !ifArmFront/t) {
                armRotate.setPosition(0);//front/down
                armTurn.setPosition(0);
                ifArmFront=true;
            }*/

//            if (this.gamepad2.b&&armPosition==0) {
//                armRotate.setPosition(0.4);//bacl
//                armPosition=1;
//            }
//            else if (this.gamepad2.b&&armPosition==1) {
//                armRotate.setPosition(0.7);//back/up
//                armPosition=2;
//            }
//            else if (this.gamepad2.x&&armPosition==2) {
//                armRotate.setPosition(0.4);//front
//                armPosition=1;
//            }
//            else if(this.gamepad2.x&&armPosition==1){
//                armRotate.setPosition(0.3);//front/down
//                armTurn.setPosition(0.78);//facing down
//                armPosition=0;
//            }
            if (this.gamepad2.b){
                armRotate.setPosition(0.68);//back/up
            }
            else if(this.gamepad2.dpad_down){
                armRotate.setPosition(0.74);//back/up
            }
            else if(this.gamepad2.x) {
                armRotate.setPosition(0.37);//front/down
                armTurn.setPosition(0.78);//facing down
            }


            //arm turn
            if (this.gamepad2.left_bumper) {
                armTurn.setPosition(0.11);//facing down
            }
            else if (this.gamepad2.right_bumper) {
                armTurn.setPosition(0.78);//facing up
            }


            // Show the elapsed game time and speed.
            telemetry.addData("Status", "Run Time: " + getRuntime());//runtime.toString()
            telemetry.addData("Speed:", speed);
            telemetry.addData("Current position(vertical slides left): ", vertarmLeft.getCurrentPosition());
            telemetry.addData("Current position(vertical slides right): ", vertarmRight.getCurrentPosition());
            telemetry.addData("Current position(horizontal slides right): ", horarm.getCurrentPosition());
            telemetry.addData("Current vertical slides place: ", vertarmPlace);
            telemetry.addData("Current arm place: ", armPosition);
            telemetry.addData("Current arm place: ", armGrabber.getPosition());

            //telemetry.addData("Current position(claw rotate): ", clawRotate.getPosition());
            telemetry.update();
        }
    }
}