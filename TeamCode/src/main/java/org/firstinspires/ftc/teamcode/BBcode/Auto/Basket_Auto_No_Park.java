package org.firstinspires.ftc.teamcode.BBcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders.RedBasketPose;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.WristClawActions;
import org.firstinspires.ftc.teamcode.BBcode.OpModeType;
import org.firstinspires.ftc.teamcode.BBcode.PoseStorage;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Locale;


@Config
//@Disabled
@Autonomous(name = "Basket_Auto_No_Park", group = "Autonomous")
@Disabled
public class Basket_Auto_No_Park extends LinearOpMode {

    @Override
    public void runOpMode() {
        //Initialization steps
        PoseStorage.previousOpMode = OpModeType.AUTONOMOUS;
        //Creates instance of MechanismActionBuilders
        WristClawActions _WristClawActions = new WristClawActions(this);
        ViperArmActions _ViperArmActions = new ViperArmActions(this);

        //Write the public FTCDesktop static fields back into the private static poses so FTCDesktop actually affects the values on restart of op mode

        //Initializes drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, RedBasketPose.init);

        //closes claw on init
        Actions.runBlocking(_WristClawActions.CloseClaw());
        Actions.runBlocking(_WristClawActions.WristDown());

        telemetry.update();
        waitForStart();
        //----------------------------------------------------------------------------------------------

        if (isStopRequested()) return;

        //drive to drop
        Action driveToDropFromStart = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(RedBasketPose.drop.position, RedBasketPose.drop.heading)
                .build();

        Action driveToDropFromInnerSample = drive.actionBuilder(RedBasketPose.inner_sample)
                .setReversed(true)
                .strafeToLinearHeading(RedBasketPose.drop.position, RedBasketPose.drop.heading)
                .build();

        Action driveToDropFromMiddleSample = drive.actionBuilder(RedBasketPose.middle_sample)
                .setReversed(true)
                .strafeToLinearHeading(RedBasketPose.drop.position, RedBasketPose.drop.heading)
                .build();

        Action driveToDropFromOuterSample = drive.actionBuilder(RedBasketPose.outer_sample)
                .setReversed(true)
                .strafeToLinearHeading(RedBasketPose.drop.position, RedBasketPose.drop.heading)
                .build();

        //sample pickup
        Action samplePickupInner = drive.actionBuilder(RedBasketPose.drop)
                .strafeToLinearHeading(RedBasketPose.inner_sample.position, RedBasketPose.inner_sample.heading)
                .build();
        Action samplePickupMiddle = drive.actionBuilder(RedBasketPose.drop)
                .strafeToLinearHeading(RedBasketPose.middle_sample.position, RedBasketPose.middle_sample.heading)
                .build();
        Action samplePickupOuter = drive.actionBuilder(RedBasketPose.drop)
                .strafeToLinearHeading(RedBasketPose.outer_sample.position, RedBasketPose.outer_sample.heading)
                .build();

//        Action submersiblePark = drive.actionBuilder(RedBasketPose.submersiblePark)
//                .strafeToLinearHeading(RedBasketPose.submersiblePark.position, RedBasketPose.submersiblePark.heading)
//                .build();
        //----------------------------------------------------------------------------------------------
        Actions.runBlocking(
                new SequentialAction(
                        _WristClawActions.WristUp(),
                        _ViperArmActions.MoveViperHalfExtend(),
                        driveToDropFromStart,
                        _ViperArmActions.DumpInHighBasketHalfExtend(),
                        samplePickupInner,
                        _WristClawActions.PickUpSample(),
                        driveToDropFromOuterSample,
                        _ViperArmActions.DumpInHighBasketHalfExtend(),
                        samplePickupMiddle,
                        _WristClawActions.CloseClaw(),
                        _WristClawActions.PickUpSample(),
                        _WristClawActions.WristUp(),
                        driveToDropFromMiddleSample,
                        _ViperArmActions.DumpInHighBasketHalfExtend(),
                        samplePickupOuter,
                        _WristClawActions.CloseClaw(),
                        _WristClawActions.PickUpSample(),
                        _WristClawActions.WristUp(),
                        driveToDropFromInnerSample,
                        _ViperArmActions.DumpInHighBasket(),
                        _WristClawActions.WristUp()
                )
        );

        PoseStorage.currentPose = drive.localizer.getPose();
        telemetry.addData("Stored Pose: ", String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", PoseStorage.currentPose.position.x, PoseStorage.currentPose.position.y, Math.toDegrees(PoseStorage.currentPose.heading.toDouble())) );
        //odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        //odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();
//        while(opModeIsActive()) {
//            // _leftFront.setPower(0.3);
//            odo.update();
//            Pose2d pos = odo.getPositionRR();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.position.x, pos.position.y, Math.toDegrees(pos.heading.toDouble()));
//
//            telemetry.update();
//        }
    }
}
