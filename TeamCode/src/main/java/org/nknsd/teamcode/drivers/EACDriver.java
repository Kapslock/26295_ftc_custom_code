package org.nknsd.teamcode.drivers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.components.handlers.ExtensionHandler;
import org.nknsd.teamcode.components.utility.GamePadHandler;
import org.nknsd.teamcode.components.handlers.IntakeSpinnerHandler;
import org.nknsd.teamcode.components.handlers.RotationHandler;
import org.nknsd.teamcode.controlSchemes.abstracts.EACControlScheme;

public class EACDriver implements NKNComponent {
    private GamePadHandler gamePadHandler;
    private RotationHandler rotationHandler;
    private ExtensionHandler extensionHandler;
    private IntakeSpinnerHandler servoHandler;
    private EACControlScheme controlScheme;

    private boolean isInEACState = true;


    Runnable rotateUp = new Runnable() {
        @Override
        public void run() {
            int nextIndex = rotationHandler.targetRotationPosition.ordinal() + 1;

            if (nextIndex >= RotationHandler.MAX_INDEX_OF_ROTATION_POSITIONS) {
                return;
            }

            rotationHandler.setTargetRotationPosition(RotationHandler.RotationPositions.values()[nextIndex]);
        }
    };
    Runnable rotateDown = new Runnable() {
        @Override
        public void run() {
            int prevIndex = rotationHandler.targetRotationPosition.ordinal() - 1;

            if (prevIndex < 0) {
                return;
            }

            rotationHandler.setTargetRotationPosition(RotationHandler.RotationPositions.values()[prevIndex]);
        }
    };
    Runnable extend = new Runnable() {
        @Override
        public void run() {
            boolean done = false; // Repeat until we either hit the end of the array or we reach a valid extension position
            int index = extensionHandler.targetPosition().ordinal();
            while (!done) {
                index++;

                if (index >= ExtensionHandler.ExtensionPositions.values().length) {
                    return;
                }

                done = extensionHandler.gotoPosition(ExtensionHandler.ExtensionPositions.values()[index]);
            }
        }
    };
    Runnable retract = new Runnable() {
        @Override
        public void run() {
            boolean done = false; // Repeat until we either hit the end of the array or we reach a valid extension position
            int index = extensionHandler.targetPosition().ordinal();
            while (!done) {
                index--;

                if (index < 0) {
                    return;
                }

                done = extensionHandler.gotoPosition(ExtensionHandler.ExtensionPositions.values()[index]);
            }
        }
    };
    Runnable grab = new Runnable() {
        @Override
        public void run() {
            servoHandler.setServoPower(IntakeSpinnerHandler.HandStates.GRIP);
        }
    };
    Runnable release = new Runnable() {
        @Override
        public void run() {
            servoHandler.setServoPower(IntakeSpinnerHandler.HandStates.RELEASE);
        }
    };
    Runnable neutral = new Runnable() {
        @Override
        public void run() {
            servoHandler.setServoPower(IntakeSpinnerHandler.HandStates.REST);
        }
    };


    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {
        // Add event listeners
        gamePadHandler.addListener(controlScheme.sampleDown(), rotateDown, "Sample Rotate Down");
        gamePadHandler.addListener(controlScheme.sampleUp(), rotateUp, "Sample Rotate Up");
        gamePadHandler.addListener(controlScheme.sampleExtend(), extend, "Sample Extend");
        gamePadHandler.addListener(controlScheme.sampleRetract(), retract, "Sample Retract");
        gamePadHandler.addListener(controlScheme.sampleGrab(), grab, "Sample Grab");
        gamePadHandler.addListener(controlScheme.sampleRelease(), release, "Sample Release");
        gamePadHandler.addListener(controlScheme.sampleNeutral(), neutral, "Sample Neutral");
        gamePadHandler.addListener(controlScheme.swapEACcontrol(), () -> isInEACState = !isInEACState, "Swap EAC & Specimen");
    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "EACDriver";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {

    }


    @Override
    public void doTelemetry(Telemetry telemetry) {
//        telemetry.addData("Rot Target", rotationHandler.targetRotationPosition.name());
//        telemetry.addData("Ext Target", extensionHandler.targetPosition().name());
//        telemetry.addData("Servo State", servoHandler.getServoPower());
        if (isInEACState) {
            telemetry.addData("Arm Controller State", "EAC");
        } else {
            telemetry.addData("Arm Controller State", "Specimen");
        }
        telemetry.addData("Sample Controls", controlScheme.getName());
    }

    public void link(GamePadHandler gamePadHandler, RotationHandler rotationHandler, ExtensionHandler extensionHandler, IntakeSpinnerHandler servoHandler, EACControlScheme controlScheme) {
        this.gamePadHandler = gamePadHandler;
        this.rotationHandler = rotationHandler;
        this.extensionHandler = extensionHandler;
        this.servoHandler = servoHandler;
        this.controlScheme = controlScheme;
    }
}
