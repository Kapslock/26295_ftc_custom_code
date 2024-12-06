package org.nknsd.teamcode.programs.autos;

import android.widget.GridLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.nknsd.teamcode.autoSteps.AutoStepAbsoluteControl;
import org.nknsd.teamcode.autoSteps.AutoStepExtendSpecimenArm;
import org.nknsd.teamcode.autoSteps.AutoStepSpecimenClaw;
import org.nknsd.teamcode.autoSteps.AutoStepSpecimenRotate;
import org.nknsd.teamcode.components.handlers.SpecimenClawHandler;
import org.nknsd.teamcode.components.handlers.SpecimenExtensionHandler;
import org.nknsd.teamcode.components.handlers.SpecimenRotationHandler;
import org.nknsd.teamcode.frameworks.NKNAutoStep;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.autoSteps.AutoStepExtendArm;
import org.nknsd.teamcode.autoSteps.AutoStepMove;
import org.nknsd.teamcode.autoSteps.AutoStepRotateArm;
import org.nknsd.teamcode.autoSteps.AutoStepServo;
import org.nknsd.teamcode.autoSteps.AutoStepSleep;
import org.nknsd.teamcode.components.handlers.ExtensionHandler;
import org.nknsd.teamcode.components.sensors.FlowSensor;
import org.nknsd.teamcode.components.sensors.IMUSensor;
import org.nknsd.teamcode.components.handlers.IntakeSpinnerHandler;
import org.nknsd.teamcode.components.sensors.PotentiometerSensor;
import org.nknsd.teamcode.components.handlers.RotationHandler;
import org.nknsd.teamcode.components.handlers.WheelHandler;
import org.nknsd.teamcode.components.utility.AutoHeart;
import org.nknsd.teamcode.frameworks.NKNProgramTrue;
import org.nknsd.teamcode.helperClasses.AutoSkeleton;

import java.util.LinkedList;
import java.util.List;

@Autonomous(name = "Score Specimen on Bar (IN DEV)")
public class SpecimenAuto extends NKNProgramTrue {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Step List
        List<NKNAutoStep> stepList = new LinkedList<NKNAutoStep>();


        // Core mover
        AutoSkeleton autoSkeleton = new AutoSkeleton(0.5, 0.8, 1.5);

        AutoHeart autoHeart = new AutoHeart(stepList);
        components.add(autoHeart);
        telemetryEnabled.add(autoHeart);


        // Sensors
        FlowSensor flowSensor = new FlowSensor();
        components.add(flowSensor);
        telemetryEnabled.add(flowSensor);

        IMUSensor imuSensor = new IMUSensor();
        components.add(imuSensor);
        //telemetryEnabled.add(imuComponent);

        PotentiometerSensor potentiometerSensor = new PotentiometerSensor();
        components.add(potentiometerSensor);


        // Wheel Handler
        WheelHandler wheelHandler = new WheelHandler();
        components.add(wheelHandler);


        // Arm Stuff
        RotationHandler rotationHandler = new RotationHandler ();
        components.add(rotationHandler);

        ExtensionHandler extensionHandler = new ExtensionHandler();
        components.add(extensionHandler);

        IntakeSpinnerHandler intakeSpinnerHandler = new IntakeSpinnerHandler();
        components.add(intakeSpinnerHandler);


        // Specimen Stuff
        SpecimenRotationHandler specimenRotationHandler = new SpecimenRotationHandler();
        components.add(specimenRotationHandler);
        telemetryEnabled.add(specimenRotationHandler);

        SpecimenExtensionHandler specimenExtensionHandler = new SpecimenExtensionHandler();
        components.add(specimenExtensionHandler);

        SpecimenClawHandler specimenClawHandler = new SpecimenClawHandler();
        components.add(specimenClawHandler);


        // Linking
        rotationHandler.link(potentiometerSensor, extensionHandler);
        extensionHandler.link(rotationHandler);

        autoSkeleton.link(wheelHandler, rotationHandler, extensionHandler, intakeSpinnerHandler, flowSensor, imuSensor);
        autoSkeleton.specimenLink(specimenExtensionHandler, specimenRotationHandler, specimenClawHandler);
        autoSkeleton.setOffset(new double[]{0.0, 0.0}, 180);
        assembleList(stepList, autoHeart, autoSkeleton);
    }

    private void assembleList(List<NKNAutoStep> stepList, AutoHeart autoHeart, AutoSkeleton autoSkeleton) {
        // Declare steps
        AutoStepSleep sleep = new AutoStepSleep(200);

        AutoStepAbsoluteControl moveToBar = new AutoStepAbsoluteControl(-0.4332, 1.20, 0);
        AutoStepMove closeInOnBar = new AutoStepMove(0, 0.2);
        AutoStepAbsoluteControl moveToB2 = new AutoStepAbsoluteControl(1.2, 1, 0);
        AutoStepMove moveUp = new AutoStepMove(0, 1);
        AutoStepAbsoluteControl moveTo1stSample = new AutoStepAbsoluteControl(1.94489, 2.4861, 90);
        AutoStepMove depositSample = new AutoStepMove(0, -1.801);

        AutoStepSpecimenRotate rotateToDeposit = new AutoStepSpecimenRotate(SpecimenRotationHandler.SpecimenRotationPositions.BACK);
        AutoStepSpecimenRotate rotateToCollect = new AutoStepSpecimenRotate(SpecimenRotationHandler.SpecimenRotationPositions.FORWARD);

        AutoStepExtendSpecimenArm extendToReady = new AutoStepExtendSpecimenArm(SpecimenExtensionHandler.SpecimenExtensionPositions.SPECIMEN_READY);
        AutoStepExtendSpecimenArm extendToClip = new AutoStepExtendSpecimenArm(SpecimenExtensionHandler.SpecimenExtensionPositions.SPECIMEN_CLIP);
        AutoStepExtendSpecimenArm extendToRest = new AutoStepExtendSpecimenArm(SpecimenExtensionHandler.SpecimenExtensionPositions.RESTING);

        AutoStepSpecimenClaw grip = new AutoStepSpecimenClaw(SpecimenClawHandler.ClawPositions.GRIP);
        AutoStepSpecimenClaw release = new AutoStepSpecimenClaw(SpecimenClawHandler.ClawPositions.RELEASE);


        // Create path
        // Approach bar and align arm
        stepList.add(grip);
        stepList.add(moveToBar);

        // Deposit 1st specimen
        stepList.add(extendToReady);
        stepList.add(closeInOnBar);
        stepList.add(extendToClip);
        stepList.add(release);

        // Transition
        stepList.add(rotateToCollect);
        stepList.add(extendToRest);

        // Push 1st blue
        stepList.add(moveToB2);
        stepList.add(moveUp);
        stepList.add(moveTo1stSample);
        stepList.add(depositSample);

        // Push 2nd blue

        // Push.. 3rd.. blue

        // Grab 2nd specimen


        autoHeart.linkSteps(stepList, autoSkeleton);
    }
}
