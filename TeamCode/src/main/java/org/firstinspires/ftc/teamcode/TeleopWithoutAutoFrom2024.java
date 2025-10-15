/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// Directory of package
package org.firstinspires.ftc.teamcode;

// Declare imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleopWithoutAuto", group="Linear OpMode")
public class TeleopWithoutAutoFrom2024 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;
    DcMotor arm = null;
    DcMotor viper = null;
    CRServo intake       = null; //the claw servo
    Servo    wrist       = null; //the wrist servo

    //times 5 because the gear ratio is 100:20
    final double TICKS_FULL_ROTATION_ARM = 1425.1 * 5;
    final double TICKS_FULL_ROTATION_VIPER =  384.5;
    final double TICKS_ONE_DEGREE = TICKS_FULL_ROTATION_ARM /360;
    double newTargetPosition;
    //divied by 4.75 because it is the number of inches per 1 rev of the motor
    final double TICKS_ONE_INCH = TICKS_FULL_ROTATION_VIPER / 4.75;



    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 240;
    final double ARM_CLEAR_BARRIER         = 230;
    final double ARM_SCORE_SPECIMEN        = 160;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 162;
    final double ARM_SCORE_SAMPLE_IN_HIGH  = 148;
    static final int ARM_HANG = 22;
    static final int ARM_STRAIGHT_UP_A2 = 108;
    static final int ARM_SAFETY_POSITION = 11;
    static final int ARM_ADJUST_DEG = 1;
    static final int VIPER_ADJUST_INCH = 1;
    final double VIPER_INTAKE = 5;
    final double VIPER_LOW_BASKET = 0;
    final double VIPER_HIGH_BASKET = 19;
    final double VIPER_SAFETY_POSITION = 0;
    final double DRIVE_POSITION = 108;
    final double TRIGGER_THRESHOLD = 0.3;
    final double INIT_ARM_LEN = 21;
    final double MAX_ARM_SHADOW = 21;
    final int ARM_HOR_DEG = 200;
    final double ARM_BACK = 126;

    // Declare constant
    // ALL CAPS MEANS DO NOT CHANGE VARIABLE
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;

    /*
    static final int CORRECTION_DEG = 4;

    static final int PICK_UP_DEG = 253 + CORRECTION_DEG;

    static final int CLEAR_BARRIER_DEG = 240 + CORRECTION_DEG;

    static final int SCORE_POSITION_DEG = 160 + CORRECTION_DEG;


    */

    int armTargetDeg;
    int viperTargetlength;

    double driveSpeed = 0.7;
    double turnSpeed = 0.6;

    boolean rampUp = true;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
        arm = hardwareMap.get(DcMotor.class,"arm");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viper = hardwareMap.get(DcMotor.class,"viper");
        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperTargetlength = 0;
        viper.setDirection(DcMotor.Direction.REVERSE);
        intake.setPower(INTAKE_OFF);


        // Initialize servo variables

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (t     he wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // Set mode for motors

        // winch.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTargetDeg = ARM_SAFETY_POSITION;
        moveArmPosition(armTargetDeg);
        wrist.setPosition(WRIST_FOLDED_IN);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;

            // Speed control
            if (gamepad1.y) {
                driveSpeed = 1;
                turnSpeed = 1;
            }
            if (gamepad1.b) {
                driveSpeed = 0.7;
                turnSpeed = 0.6;
            }
            if (gamepad1.a) {
                driveSpeed = 0.4;
                turnSpeed = 0.3;
            }
            //intake control
            if (gamepad2.dpad_up) {
                intake.setPower(INTAKE_DEPOSIT);
            }
            if (gamepad2.dpad_left) {
                intake.setPower(INTAKE_OFF);
            }
            if (gamepad2.dpad_down) {
                intake.setPower(INTAKE_COLLECT);
            }



            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw =
                    gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = driveSpeed * (axial + lateral) + turnSpeed * yaw;
            double rightFrontPower = driveSpeed * (axial - lateral) - turnSpeed * yaw;
            double leftBackPower = driveSpeed * (axial - lateral) + turnSpeed * yaw;
            double rightBackPower = driveSpeed * (axial + lateral) - turnSpeed * yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // if gamepad 1's a button is pressed and when we get the position of our arm servo. If the value for our position is not
            // equal to END_POS_YP, then we will set the servo arm position to START_POS_YP.


            if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
                viperTargetlength = (int) VIPER_SAFETY_POSITION;
                armTargetDeg = (int) DRIVE_POSITION;
                moveViperSlide(viperTargetlength, armTargetDeg);
                sleep(1000); //FIXME Test retraction time and adjust
                moveArmPosition(armTargetDeg);
            }
            //Space


            if (gamepad2.x) {
                //Move Viper Slide 238 degrees
                //rotate wrist to position .50
                //Move Viper Slide 5in
                //rotate intake clockwise continuously
                armTargetDeg = (int)ARM_COLLECT;
                moveArmPosition(armTargetDeg);
                sleep(1000);
                wrist.setPosition(WRIST_FOLDED_OUT);
                sleep(500);
                viperTargetlength = (int) VIPER_INTAKE;
                moveViperSlide(viperTargetlength, armTargetDeg);
                //intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad2.b) {
                armTargetDeg = (int)ARM_CLEAR_BARRIER;
                moveArmPosition(armTargetDeg);
                viperTargetlength = 0;
                moveViperSlide(viperTargetlength, armTargetDeg);
                intake.setPower(INTAKE_OFF);
            }
            else if (gamepad2.a) {
                armTargetDeg = (int)ARM_SCORE_SAMPLE_IN_LOW;
                moveArmPosition(armTargetDeg);
                sleep(1000);
                //wrist.setPosition(WRIST_FOLDED_OUT);
                //sleep(1000);
                viperTargetlength = (int) VIPER_LOW_BASKET;
                moveViperSlide(viperTargetlength, armTargetDeg);
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_OUT);
            }
            else if (gamepad2.y) {
                armTargetDeg = (int)ARM_SCORE_SAMPLE_IN_HIGH;
                moveArmPosition(armTargetDeg);
                sleep(1000);
                //wrist.setPosition(WRIST_FOLDED_OUT);
                //sleep(1000);
                viperTargetlength = (int) VIPER_HIGH_BASKET;
                moveViperSlide(viperTargetlength, armTargetDeg);
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_OUT);
            }
            else if (gamepad2.dpad_right) {
                intake.setPower(INTAKE_OFF);
                viperTargetlength = (int) VIPER_SAFETY_POSITION;
                moveViperSlide(viperTargetlength, armTargetDeg);
                sleep(2000);
                wrist.setPosition(WRIST_FOLDED_IN);
                armTargetDeg = ARM_SAFETY_POSITION;
                moveArmPosition(armTargetDeg);

            }
            else if (gamepad2.left_bumper) { //straight up to prepare for A2
                intake.setPower(INTAKE_OFF);
                viperTargetlength = (int) VIPER_SAFETY_POSITION;
                moveViperSlide(viperTargetlength, armTargetDeg);
                sleep(2000);
                wrist.setPosition(WRIST_FOLDED_IN);
                armTargetDeg = ARM_STRAIGHT_UP_A2;
                moveArmPosition(armTargetDeg);
            }
            else if (gamepad2.left_trigger > 0.3) {
                intake.setPower(INTAKE_OFF);
                armTargetDeg = ARM_HANG;
                moveArmPosition(armTargetDeg);

            }

            else if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                armTargetDeg = armTargetDeg - ARM_ADJUST_DEG;
                moveArmPosition(armTargetDeg);
            }
            else if (gamepad1.left_bumper) {
                armTargetDeg = armTargetDeg + ARM_ADJUST_DEG;
                moveArmPosition(armTargetDeg);
            }

            if (gamepad1.right_bumper){
                viperTargetlength = viperTargetlength - VIPER_ADJUST_INCH;
                moveViperSlide(viperTargetlength, armTargetDeg);
            }
            else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                viperTargetlength = viperTargetlength + VIPER_ADJUST_INCH;
                moveViperSlide(viperTargetlength, armTargetDeg);
            }
            else if (gamepad1.dpad_left) {
                moveViperSlide(0, armTargetDeg);
            }


            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
    public void moveArmPosition(int newTargetDegrees){
        double newTargetPosition = newTargetDegrees * TICKS_ONE_DEGREE;
        arm.setTargetPosition((int)newTargetPosition);

        arm.setPower(0.5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine("done with encoder method: " + newTargetDegrees);
        telemetry.update();
        sleep(100);
    }
    public void moveViperSlide(double newTargetExtensionInches, int currArmPos) {
       double newTargetPosition;
       double maxExt = 0;
       int armAngle = -1;
       double maxArmLen = INIT_ARM_LEN;
        telemetry.addLine("MOVING VIPER SLIDE: " + currArmPos + "  " + newTargetExtensionInches);
        telemetry.update();
        //sleep(1000);
        if (currArmPos < ARM_BACK) {
            maxExt = 0;
            telemetry.addLine("arm BACK at: " + currArmPos + "  " + armAngle);
            telemetry.addLine("max arm length: " + maxArmLen + "  " + maxExt);
            telemetry.update();
            //sleep(1000);
        }
        else {
            armAngle = Math.abs(ARM_HOR_DEG - currArmPos);
            maxArmLen = INIT_ARM_LEN / Math.cos(armAngle * Math.PI / 180);
            maxExt = maxArmLen - INIT_ARM_LEN;
            maxExt = Math.max(maxExt,0);

            telemetry.addLine("arm forward at: " + currArmPos + " arm angle: " + armAngle);
            telemetry.addLine("max arm length: " + maxArmLen + " max extension: " + maxExt);
            telemetry.update();
            //sleep(1000);
        }

        newTargetExtensionInches = Math.min(newTargetExtensionInches, maxExt);
        newTargetPosition = newTargetExtensionInches * TICKS_ONE_INCH;
        viper.setTargetPosition((int) newTargetPosition);

        // arm.setTargetPosition((int)100);

        viper.setPower(0.5);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine("done with encoder method: " + newTargetExtensionInches);
        telemetry.update();
        //sleep(100);
    }
}





