package org.firstinspires.ftc.teamcode.autoshellclasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders.RedBasketPose;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.ViperArmActions;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.WristClawActions;
import org.firstinspires.ftc.teamcode.BBcode.OpModeType;
import org.firstinspires.ftc.teamcode.BBcode.PoseStorage;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Locale;


@Config
@Autonomous(name = "New_Clipping_Method_Auto", group = "Autonomous")
public class New_Clipping_Method_Auto extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Initialization steps
        PoseStorage.previousOpMode = OpModeType.AUTONOMOUS;
        PoseStorage.currentPose = RedBasketPose.basket_init_old; //This is to reset the pose te a default value that is not affected by where the auto ends so that teleop opmodes can be debugged
        //Creates instance of MechanismActionBuilders
        WristClawActions _WristClawActions = new WristClawActions(this);
        ViperArmActions _ViperArmActions = new ViperArmActions(this);

        //Initializes drive
        Pose2d initialPose = new Pose2d(7.5, -61.85, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //closes claw on init
        Actions.runBlocking(_WristClawActions.CloseClaw());
        //raises wrist on init
        Actions.runBlocking(_WristClawActions.WristDown());

        telemetry.update();
        waitForStart();
        //----------------------------------------------------------------------------------------------

        if (isStopRequested()) return;


        Action clippingSpecimen1, sampleGrab1, sampleDrop1, sampleGrab2, sampleDrop2, grabSpecimen2, driveToClip2, clippingSpecimen2, grabSpecimen3, driveToClip3, clippingSpecimen3, grabSpecimen4, driveToClip4, clippingSpecimen4, driveToPark, clawCloseSpecimenWait1, clawCloseSpecimenWait2, clawCloseSpecimenWait3, clawCloseSampleWait1, clawCloseSampleWait2, clawOpenSampleWait1, clawOpenSampleWait2, viperOutWait, specimenFlipWaitTime, sampleTurnWait2, clawOpenSpecimenWait1, clawOpenSpecimenWait2, clawOpenSpecimenWait3, clawOpenSpecimenWait4, wristUpWait1, wristUpWait2, wristUpWait3, wristUpWait4, viperOutClipWait1,viperOutClipWait2, viperOutClipWait3, viperOutClipWait4, waitToClip1, waitToClip2, waitToClip3;

        clippingSpecimen1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(3,-28.5),Math.toRadians(-90))
                .build();
        clippingSpecimen2 = drive.actionBuilder(new Pose2d(45.72,-48.73,Math.toRadians(-85)))
                .afterTime(0.25, _ViperArmActions.MoveArmToNewClipMethod())
                .strafeToLinearHeading(new Vector2d(2,-28.5),Math.toRadians(-90))
                .build();
        clippingSpecimen3 = drive.actionBuilder(new Pose2d(38,-60,Math.toRadians(0)))
                .afterTime(0.25, _ViperArmActions.MoveArmToNewClipMethod())
                .strafeToLinearHeading(new Vector2d(1,-28.5),Math.toRadians(-90))
                .build();
        clippingSpecimen4 = drive.actionBuilder(new Pose2d(38,-60,Math.toRadians(0)))
                .afterTime(0.25, _ViperArmActions.MoveArmToNewClipMethod())
                .strafeToLinearHeading(new Vector2d(0,-28.5),Math.toRadians(-90))
                .build();

        sampleGrab1 = drive.actionBuilder(new Pose2d(3,-28.5,Math.toRadians(-90)))
                .afterTime(1, _ViperArmActions.MoveViperToLongSamplePickUp())
                .splineToLinearHeading(new Pose2d(34.37,-48, Math.toRadians(58.48) ), Math.toRadians(90), new TranslationalVelConstraint(75))
                .build();
        sampleGrab2 = drive.actionBuilder(new Pose2d(34.37,-48,Math.toRadians(-38)))
                .strafeToLinearHeading(new Vector2d(47.82,-50.29), Math.toRadians(66.38))
                .build();

        sampleDrop1 = drive.actionBuilder(new Pose2d(34.37,-48,Math.toRadians(58.48)))
                .turnTo(Math.toRadians(-38))
                .build();
        sampleDrop2 = drive.actionBuilder(new Pose2d(47.82,-50.29,Math.toRadians(66.38)))
                .turnTo(Math.toRadians(-40))
                .build();

        grabSpecimen2 = drive.actionBuilder(new Pose2d(47.82,-50.29,Math.toRadians(-40)))
                .strafeToLinearHeading(new Vector2d(47.68,-48.5), Math.toRadians(-85), new TranslationalVelConstraint(25))
                .build();

        grabSpecimen3 = drive.actionBuilder(new Pose2d(2,-28.5,Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(38,-60), Math.toRadians(0))
                .build();
        grabSpecimen4 = drive.actionBuilder(new Pose2d(1,-28.5,Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(38,-60), Math.toRadians(0))
                .build();

        driveToPark = drive.actionBuilder(new Pose2d(0,-28.5,Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(60,-60),Math.toRadians(90))
                .build();

        clawCloseSpecimenWait1 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.3)
                .build();
        clawCloseSpecimenWait2 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.3)
                .build();
        clawCloseSpecimenWait3 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.3)
                .build();

        clawCloseSampleWait1 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.4)
                .build();
        clawCloseSampleWait2 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.4)
                .build();

        clawOpenSampleWait1 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.2)
                .build();
        clawOpenSampleWait2 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.35)
                .build();

        clawOpenSpecimenWait1 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.2)
                .build();
        clawOpenSpecimenWait2 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.2)
                .build();
        clawOpenSpecimenWait3 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.2)
                .build();
        clawOpenSpecimenWait4 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.2)
                .build();

        wristUpWait1 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.25)
                .build();
        wristUpWait2 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.25)
                .build();
        wristUpWait3 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.25)
                .build();

        viperOutWait = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.35)
                .build();

        viperOutClipWait1 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.4)
                .build();
        viperOutClipWait2 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.4)
                .build();
        viperOutClipWait3 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.4)
                .build();
        viperOutClipWait4 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.4)
                .build();

        specimenFlipWaitTime = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.2)
                .build();
        sampleTurnWait2 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.75)
                .build();

        waitToClip1 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.2)
                .build();
        waitToClip2 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.2)
                .build();
        waitToClip3 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.2)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        _WristClawActions.WristUp(),
                        _ViperArmActions.MoveArmToNewClipMethod(),
                        specimenFlipWaitTime,
                        clippingSpecimen1,
                        _ViperArmActions.MoveViperToNewClipMethod(),
                        viperOutClipWait1,
                        _WristClawActions.OpenClaw(),
                        clawOpenSpecimenWait1,
                        _ViperArmActions.MoveArmToFastHome(),
                        _ViperArmActions.MoveViperToHome(),
                        _WristClawActions.WristDown(),
                        sampleGrab1,
                        _WristClawActions.CloseClaw(),
                        clawCloseSampleWait1,
                        sampleDrop1,
                        _WristClawActions.OpenClaw(),
                        clawOpenSampleWait1,
                        sampleGrab2,
                        _WristClawActions.CloseClaw(),
                        clawCloseSampleWait2,
                        _WristClawActions.WristSpecimenPickup(),
                        _ViperArmActions.MoveViperToShortExtend(),
                        sampleDrop2,
                        _WristClawActions.OpenClaw(),
                        clawOpenSampleWait2,
                        grabSpecimen2,
                        _WristClawActions.CloseClaw(),
                        clawCloseSpecimenWait1,
                        _WristClawActions.WristUp(),
                        wristUpWait1,
                        _ViperArmActions.MoveViperToHome(),
                        clippingSpecimen2,
                        waitToClip1,
                        _ViperArmActions.MoveViperToNewClipMethod(),
                        viperOutClipWait2,
                        _WristClawActions.OpenClaw(),
                        clawOpenSpecimenWait2,
                        _ViperArmActions.MoveArmToFastHome(),
                        _ViperArmActions.MoveViperToHome(),
                        _WristClawActions.WristSpecimenPickup(),
                        grabSpecimen3,
                        _WristClawActions.CloseClaw(),
                        clawCloseSpecimenWait2,
                        _WristClawActions.WristUp(),
                        wristUpWait2,
                        clippingSpecimen3,
                        waitToClip2,
                        _ViperArmActions.MoveViperToNewClipMethod(),
                        viperOutClipWait3,
                        _WristClawActions.OpenClaw(),
                        clawOpenSpecimenWait3,
                        _ViperArmActions.MoveArmToFastHome(),
                        _ViperArmActions.MoveViperToHome(),
                        _WristClawActions.WristSpecimenPickup(),
                        grabSpecimen4,
                        _WristClawActions.CloseClaw(),
                        clawCloseSpecimenWait3,
                        _WristClawActions.WristUp(),
                        wristUpWait3,
                        clippingSpecimen4,
                        waitToClip3,
                        _ViperArmActions.MoveViperToNewClipMethod(),
                        viperOutClipWait4,
                        _WristClawActions.OpenClaw(),
                        clawOpenSpecimenWait4,
                        _ViperArmActions.MoveArmToFastHome(),
                        _ViperArmActions.MoveViperToHome(),
                        driveToPark
                )
        );

        PoseStorage.currentPose = drive.localizer.getPose(); //save the pose for teleop
        telemetry.addData("Stored Pose: ", String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", PoseStorage.currentPose.position.x, PoseStorage.currentPose.position.y, Math.toDegrees(PoseStorage.currentPose.heading.toDouble())) );

        while(opModeIsActive()) {
            // _leftFront.setPower(0.3);
            telemetry.update();
        }
    }
}
