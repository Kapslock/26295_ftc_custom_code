package org.firstinspires.ftc.teamcode.BBcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders.SpecimenPose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Arm;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.ChristmasLight;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Viper;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.WristClaw;

import java.util.Locale;

@TeleOp(name = " New Clip Method TeleOp")
public class NewClipMethodInTeleop extends LinearOpMode{
    enum HighBasketState {
        Home,
        RisingArmSample,
        ViperExtendFull,
        WristDump,
        HighBasket,
        WristUp,
        ViperClosed,
        ViperRetractedShort,
        LoweringArm,
        ArmDown
    }

    enum HangState {
        Home,
        RaiseArmHang,
        ViperExtendHang,
        WristDown,
        Hang,
        ViperDown,
        ArmDown,
        ControlledLetDown
    }

    enum SpecimenClipState {
        Home,
        WristUp,
        RaiseArm,
        ViperExtend,
        SpecimenHang,
        ClawOpen,
        ViperClose,
        LowerArm,
        SpecimenPickup
    }

    enum SubmersiblePickupState {
        Home,
        LongWristUpIn,
        ShortWristUpIn,
        LongExtendViperPickup,
        ShortExtendViperPickup,
        SubmersiblePickup,
        WristUpOut,
        ExtendViperShort,
        ExtendViperClosed
    }


    HighBasketState highBasketState = HighBasketState.Home;
    HangState hangState = HangState.Home;
    SpecimenClipState specimenClipState = SpecimenClipState.Home;
    SubmersiblePickupState submersiblePickupState = SubmersiblePickupState.Home;
    Limelight3A _limelight;
    ElapsedTime wristTimer = new ElapsedTime();
    ElapsedTime controlledLetDownTimer = new ElapsedTime();

    final double wristFlipTime = 0.5;
    final double controlledLetDownTime = 15;
    Pose2d botPose = new Pose2d(0, 0, 0);

    private void handleGamepad1 (Viper viper, WristClaw wristClaw) {
        //Specimen Pickup Position
        if (gamepad1.x) {
            wristClaw.WristSpecimenPickup();
        }
        //Inspection wrist setup
//        if (gamepad1.b) {
//            wristClaw.WristMid();
//        }
    }

    private void handleGamepad2 (WristClaw wristClaw) {

        //Open Claw
        if(gamepad2.b) {
            telemetry.update();
            wristClaw.OpenClaw();
        }

        //Close Claw
        if(gamepad2.x) {
            telemetry.update();
            wristClaw.CloseClaw();
        }

        //Move Claw Up
        if(gamepad2.y) {
            wristClaw.WristUp();
        }

        //Move Claw Down
        if(gamepad2.a) {
            wristClaw.WristDown();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException{
        // Initialization Code Goes Here
        ChristmasLight _christmasLight = new ChristmasLight(this);

        //Init limelight
        _limelight = hardwareMap.get(Limelight3A.class, "limelight");
        _limelight.pipelineSwitch(1);
        _limelight.start();

        TelemetryHelper telemetryHelper = new TelemetryHelper(this);
        //Allows for telemetry to be added to without clearing previous data. This allows setting up telemetry functions to be called in the loop or adding telemetry items within a function and not having it cleared on next loop
        telemetry.setAutoClear(false);
        //Init for the other classes this opmode pulls methods from
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
        Arm arm = new Arm(this, telemetryHelper);
        Viper viper = new Viper(this);
        WristClaw wristClaw = new WristClaw(this);
        arm.Reset();
        viper.StopAndResetEncoder();
        wristTimer.reset();

        //Call the function to initialize telemetry functions
        telemetryHelper.initGamepadTelemetry(gamepad1);
        telemetryHelper.initGamepadTelemetry(gamepad2);

        //Where the start button is clicked, put some starting commands after
        boolean isDpadUpPressed = false;
        boolean isDpadDownPressed = false;
        waitForStart();
        telemetry.addData("HasRolloverPose", PoseStorage::hasRolloverPose);
        if (PoseStorage.hasRolloverPose()) {
            _christmasLight.off();
            drivetrain.localizer.setPose(PoseStorage.currentPose);
            PoseStorage.hasFieldCentricDrive = true;
        } else {
            _christmasLight.yellow();
            PoseStorage.hasFieldCentricDrive = false;
            //TODO setPose to a some other likely position??
        }

        arm.MoveToHome();

        telemetry.addData("PoseStorage", ()-> PoseStorage.currentPose);
        //telemetry.addData("LimeLightPose", () -> formatLimeLight(botPose));
        telemetry.addData("Specimen Angle (deg)", () -> arm.specimenPosition);
        telemetry.addData("Current Clip Position", () -> SpecimenPose.current_Clip);

        boolean tagFound = false;
        double botYaw = 0;
        while(opModeIsActive()){ //while loop for when program is active
            //Manage LimeLight
            LLResult lLResult = _limelight.getLatestResult();
            if (lLResult != null && lLResult.isValid()) {

                if (lLResult.getTa() > 0) {
                    tagFound = true;
                    Pose3D limeLightPose = lLResult.getBotpose();
                    botPose = new Pose2d(limeLightPose.getPosition().x*39.3701, limeLightPose.getPosition().y*39.3701, limeLightPose.getOrientation().getYaw());
                    botYaw = lLResult.getBotpose().getOrientation().getYaw();
//                    _christmasLight.green();
                    //telemetry.addData("BotPose", botPose.getPosition());
                    //telemetry.addData("Yaw", botYaw);
                }
                else {
                    tagFound = false;
//                    _christmasLight.off();
                    botPose = null;
                    //telemetry.addData("Limelight", "Tag not found");
                }
            }
            else {
                tagFound = false;
//                _christmasLight.off();
                botPose = null;
                //telemetry.addData("Limelight", "No data available");
            }
            //Drive code
            drivetrain.Drive();

            handleGamepad1(viper, wristClaw);
            handleGamepad2(wristClaw);

            switch (highBasketState) {
                case Home:
                    if (gamepad2.left_trigger > 0 && gamepad2.dpad_up) {
                        wristClaw.WristDown();
                        arm.MoveToHighBasket();
                        highBasketState = HighBasketState.RisingArmSample;
                    }
                    break;
                case RisingArmSample:
                    if (arm.getIsArmHighBasketPosition()) {
                        viper.ExtendFull(1);
                        highBasketState = HighBasketState.ViperExtendFull;
                    }
                    else if (gamepad2.left_trigger > 0 && gamepad2.dpad_down) {
                        arm.MoveToHome();
                        highBasketState = HighBasketState.LoweringArm;
                    }
                    break;

                case ViperExtendFull:
                    if (viper.getIsViperExtendFull()) {
                        wristClaw.WristUp();
//                        wristClaw.WristDump();
//                        highBasketState = HighBasketState.WristDump;
                        highBasketState = HighBasketState.HighBasket;
//                        wristTimer.reset();
                    }
                    else if (gamepad2.left_trigger > 0 && gamepad2.dpad_down) {
                        viper.ExtendShort(1);
                        highBasketState = HighBasketState.ViperRetractedShort;
                    }
                    break;

//                case WristDump:
//                    if (wristTimer.seconds() >= wristFlipTime){
//                        highBasketState = HighBasketState.HighBasket;
//                    }
//                    else if (gamepad2.left_trigger > 0 && gamepad2.dpad_down) {
//                        wristClaw.WristUp();
//                        highBasketState = HighBasketState.WristUp;
//                        wristTimer.reset();
//                    }
//                    break;

                case HighBasket:
                    if (gamepad2.left_trigger > 0 && gamepad2.dpad_down) {
//                        wristClaw.WristUp();
//                        highBasketState = HighBasketState.WristUp;
//                        wristTimer.reset();
                        viper.ExtendShort(1);
                        highBasketState = HighBasketState.ViperRetractedShort;
                    }
                    break;
//
//                case WristUp:
//                    if (wristTimer.seconds() >= wristFlipTime) {
//                        viper.ExtendShort(1);
//                        highBasketState = HighBasketState.ViperRetractedShort;
//                    }
//                    break;

                case ViperRetractedShort:
                    if (viper.getIsViperRetractedShort()) {
                        viper.ExtendClosed(0.25);
                        highBasketState = HighBasketState.ViperClosed;
                    }

                case ViperClosed:
                    if (viper.getIsViperExtendClosed()) {
                        viper.Rest();
                        arm.MoveToSlowDown();
                        highBasketState = HighBasketState.LoweringArm;
                    }
                    break;

                case LoweringArm:
                    if (arm.getIsArmSlowDownPosition()) {
                        arm.MoveToHome();
                        highBasketState = HighBasketState.ArmDown;
                    }
                    break;

                case ArmDown:
                    if (arm.getIsArmHomePosition()) {
                        highBasketState = HighBasketState.Home;
                    }
            }

            switch (hangState) {
                case Home:
                    if (gamepad1.left_trigger > 0 && gamepad1.dpad_up) {
                        arm.MoveToHangOut();
                        hangState = HangState.RaiseArmHang;
                    }
                    else if (gamepad1.left_trigger > 0 && gamepad1.dpad_down) {
                        arm.MoveToHome();
                    }
                    break;
                case RaiseArmHang:
                    if (arm.getIsArmHangOutPosition()) {
                        viper.ExtendOutHang(1);
                        hangState = HangState.ViperExtendHang;
                    }
                    break;

                case ViperExtendHang:
                    if (viper.getIsViperExtendSpecimenHang()) {
                        hangState = HangState.Hang;
                    }
                    break;

                case Hang:
                    if (gamepad1.left_trigger > 0 && gamepad1.dpad_down) {
                        arm.Stop();
                        viper.ExtendInHang(1);
                        hangState = HangState.ViperDown;
                    }
                    break;

                case ViperDown:
                    if (viper.getIsViperRetractedShort()) {
                        arm.MoveToHangIn();
                        hangState = HangState.ArmDown;
                        controlledLetDownTimer.reset();
                    }
                    break;

                case ArmDown:
                    if (controlledLetDownTimer.seconds() >= controlledLetDownTime) {
                        arm.SlowLetDown();
                        viper.SlowLetDown();
                        hangState = HangState.Home;
                    }
                    break;
            }

            switch (specimenClipState) {
                case Home:
                    if (gamepad2.right_trigger > 0 && gamepad2.dpad_up) {
                        wristClaw.WristUp();
                        specimenClipState = SpecimenClipState.WristUp;
                        wristTimer.reset();
                    }
                    break;

                case WristUp:
                    if (wristTimer.seconds() >= wristFlipTime) {
                        arm.MoveToNewClipMethodForTeleop();
                        specimenClipState = SpecimenClipState.RaiseArm;
                    }
                case RaiseArm:
                    if (arm.getIsArmNewClipMethodPosition()) {
                        specimenClipState = SpecimenClipState.SpecimenHang;
                    }
                    break;

                case SpecimenHang:
                    if (gamepad2.right_trigger > 0 && gamepad2.dpad_down) {
                        viper.ExtendNewSpecimenClip(1);
                        specimenClipState = SpecimenClipState.ViperExtend;
                    }
                    break;

                case ViperExtend:
                    if (viper.getIsViperExtendNewClipMethod()) {
                        wristClaw.OpenClaw();
                        specimenClipState = SpecimenClipState.ClawOpen;
                        wristTimer.reset();
                    }
                    break;

                case ClawOpen:
                    if (wristTimer.seconds() >= wristFlipTime) {
                        viper.ExtendClosed(0.75);
                        specimenClipState = SpecimenClipState.ViperClose;
                    }

                case ViperClose:
                    if (viper.getIsViperExtendClosed()) {
                        viper.Rest();
                        arm.MoveToFastHome();
                        specimenClipState = SpecimenClipState.LowerArm;
                    }
                    break;

                case LowerArm:
                    if (arm.getIsArmHomePosition()) {
                        arm.Stop();
                        wristClaw.WristSpecimenPickup();
                        specimenClipState = SpecimenClipState.SpecimenPickup;
                        wristTimer.reset();
                    }
                    break;

                case SpecimenPickup:
                    if (wristTimer.seconds() >= wristFlipTime) {
                        specimenClipState = SpecimenClipState.Home;
                    }
            }

            switch (submersiblePickupState) {
                case Home:
                    if (gamepad1.y) {
                        wristClaw.WristUp();
                        submersiblePickupState = SubmersiblePickupState.LongWristUpIn;
                        wristTimer.reset();
                    }
                    else if (gamepad1.b){
                        wristClaw.WristUp();
                        submersiblePickupState = SubmersiblePickupState.ShortWristUpIn;
                        wristTimer.reset();
                    }
                    break;

                case LongWristUpIn:
                    if (wristTimer.seconds() >= wristFlipTime) {
                        viper.Extendlongsubmersible(1);
                        submersiblePickupState = SubmersiblePickupState.LongExtendViperPickup;
                    }
                    break;

                case ShortWristUpIn:
                    if (wristTimer.seconds() >= wristFlipTime) {
                        viper.Extendshortsubmersible(1);
                        submersiblePickupState = SubmersiblePickupState.ShortExtendViperPickup;
                    }
                    break;

                case LongExtendViperPickup:
                    if (viper.getIsViperlongExtendSub()) {
                        submersiblePickupState = SubmersiblePickupState.SubmersiblePickup;
                    }
                    break;

                case ShortExtendViperPickup:
                    if (viper.getIsVipershortExtendSub()) {
                        submersiblePickupState = SubmersiblePickupState.SubmersiblePickup;
                    }
                    break;

                case SubmersiblePickup:
                    if (gamepad1.a) {
                        wristClaw.WristUp();
                        submersiblePickupState = SubmersiblePickupState.WristUpOut;
                        wristTimer.reset();
                    }
                    else if (gamepad1.y) {
                        viper.Extendlongsubmersible(1);
                        submersiblePickupState = SubmersiblePickupState.LongExtendViperPickup;
                    }
                    break;

                case WristUpOut:
                    if (wristTimer.seconds() >= wristFlipTime) {
                        viper.ExtendClosed(1);
                        submersiblePickupState = SubmersiblePickupState.ExtendViperShort;
                    }
                    break;

                case ExtendViperShort:
                    if (viper.getIsViperExtendClosed()) {
                        viper.Rest();
                        submersiblePickupState = SubmersiblePickupState.ExtendViperClosed;
                    }
                    break;


                case ExtendViperClosed:
                    if (viper.getIsViperRetractedShort()) {
                        submersiblePickupState = SubmersiblePickupState.Home;
                    }
            }

            //Pose2d pos = odo.getPositionRR();
            //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.position.x, pos.position.y, Math.toDegrees(pos.heading.toDouble()));


            telemetry.update();

        }

    }
    private String getPinpoint(Pose2D pos) {
        return String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), (pos.getHeading(AngleUnit.DEGREES)));
    }
    private String getPinpoint(Pose2d pos) {
        return String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.position.x, pos.position.y, Math.toDegrees(pos.heading.toDouble()));
    }
    private static String formatLimeLight(Pose2d botPose) {
        if (botPose == null) {
            return "No data available";
        }
        return String.format(Locale.US, "{X: %.2f, Y: %.2f, H: %.2f}", botPose.position.x, botPose.position.y, Math.toDegrees(botPose.heading.toDouble()));

    }
}