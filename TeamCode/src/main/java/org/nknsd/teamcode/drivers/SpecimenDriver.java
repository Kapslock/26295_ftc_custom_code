package org.nknsd.teamcode.drivers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.controlSchemes.abstracts.SpecimenControlScheme;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.components.handlers.SpecimenClawHandler;
import org.nknsd.teamcode.components.handlers.SpecimenExtensionHandler;
import org.nknsd.teamcode.components.handlers.SpecimenRotationHandler;
import org.nknsd.teamcode.components.utility.GamePadHandler;

public class SpecimenDriver implements NKNComponent {
    private GamePadHandler gamePadHandler;
    private SpecimenExtensionHandler specimenExtensionHandler;
    private SpecimenRotationHandler specimenRotationHandler;
    private SpecimenClawHandler specimenClawHandler;
    private SpecimenControlScheme controlScheme;

    Runnable specimenExtend = new Runnable() {
        @Override
        public void run() {
            boolean done = false; // Repeat until we either hit the end of the array or we reach a valid extension position
            int index = specimenExtensionHandler.targetPosition().ordinal();
            while (!done) {
                index++;

                if (index >= SpecimenExtensionHandler.SpecimenExtensionPositions.values().length) {
                    return;
                }

                done = specimenExtensionHandler.gotoPosition(SpecimenExtensionHandler.SpecimenExtensionPositions.values()[index]);
            }
        }
    };
    Runnable specimenRetract = new Runnable() {
        @Override
        public void run() {
            boolean done = false; // Repeat until we either hit the end of the array or we reach a valid rotation position
            int index = specimenExtensionHandler.targetPosition().ordinal();
            while (!done) {
                index --;

                if (index < 0) {return;}

                done = specimenExtensionHandler.gotoPosition(SpecimenExtensionHandler.SpecimenExtensionPositions.values()[index]);
            }
        }
    };
    Runnable specimenForward = new Runnable() {
        @Override
        public void run() {
            boolean done = false; // Repeat until we either hit the end of the array or we reach a valid extension position
            int index = specimenRotationHandler.targetPosition().ordinal();
            while (!done) {
                index++;

                if (index >= SpecimenRotationHandler.SpecimenRotationPositions.values().length) {
                    return;
                }

                done = specimenRotationHandler.goToPosition(SpecimenRotationHandler.SpecimenRotationPositions.values()[index]);
            }
        }
    };
    Runnable specimenBackward = new Runnable() {
        @Override
        public void run() {
            boolean done = false; // Repeat until we either hit the end of the array or we reach a valid extension position
            int index = specimenRotationHandler.targetPosition().ordinal();
            while (!done) {
                index --;

                if (index < 0) {return;}

                done = specimenRotationHandler.goToPosition(SpecimenRotationHandler.SpecimenRotationPositions.values()[index]);
            }
        }
    };
    Runnable goToRestingAfterRelease = new Runnable(){

        @Override
        public void run() {
            specimenRotationHandler.goToPosition(SpecimenRotationHandler.SpecimenRotationPositions.MIDDLE);
        }
    };
    Runnable grip = new Runnable() {
        @Override
        public void run() {
            specimenClawHandler.setClawPosition(SpecimenClawHandler.ClawPositions.GRIP);
        }
    };
    Runnable release = new Runnable() {
        @Override
        public void run() {
            specimenClawHandler.setClawPosition(SpecimenClawHandler.ClawPositions.RELEASE);
        }
    };
    private Telemetry telemetry;

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {
        gamePadHandler.addListener2(controlScheme.specimenBackwards(), specimenBackward, "Specimen Rotate Backwards");
        gamePadHandler.addListener2(controlScheme.specimenForward(), specimenForward, "Specimen Rotate Forwards");
        gamePadHandler.addListener2(controlScheme.specimenGrab(), grip, "Specimen Grip");
        gamePadHandler.addListener2(controlScheme.specimenRelease(), release, "Specimen Release");
        gamePadHandler.addListener2(controlScheme.specimenRaise(), specimenExtend, "Specimen Extend");
        gamePadHandler.addListener2(controlScheme.specimenLower(), specimenRetract, "Specimen Lower");
        gamePadHandler.addListener2(controlScheme.goToRestingAfterRelease(), goToRestingAfterRelease,"Go To Resting After Release");
    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "SpecimenDriver";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Specimen Controls", controlScheme.getName());
    }

    public void link (SpecimenExtensionHandler specimenExtensionHandler, SpecimenRotationHandler specimenRotationHandler, SpecimenClawHandler specimenClawHandler, GamePadHandler gamepadHandler, SpecimenControlScheme controlScheme){
        this.specimenExtensionHandler = specimenExtensionHandler;
        this.specimenRotationHandler = specimenRotationHandler;
        this.specimenClawHandler = specimenClawHandler;
        this.gamePadHandler = gamepadHandler;
        this.controlScheme = controlScheme;
    }
}
