package org.nknsd.teamcode.programs.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.nknsd.teamcode.controlSchemes.reals.CollyWheelController;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.components.handlers.ExtensionHandler;
import org.nknsd.teamcode.components.utility.GamePadHandler;
import org.nknsd.teamcode.components.sensors.IMUSensor;
import org.nknsd.teamcode.components.handlers.IntakeSpinnerHandler;
import org.nknsd.teamcode.components.sensors.PotentiometerSensor;
import org.nknsd.teamcode.components.handlers.RotationHandler;
import org.nknsd.teamcode.components.handlers.WheelHandler;
import org.nknsd.teamcode.drivers.CollyRandomWheelDriver;
import org.nknsd.teamcode.drivers.EACDriver;
import org.nknsd.teamcode.controlSchemes.reals.KarstenEACController;
import org.nknsd.teamcode.frameworks.NKNProgramTrue;

import java.util.List;

@TeleOp(name = "Collyien Physics")
public class CollyMovementNKNProgram extends NKNProgramTrue {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Misc
        GamePadHandler gamePadHandler = new GamePadHandler();
        components.add(gamePadHandler);

        WheelHandler wheelHandler = new WheelHandler();
        components.add(wheelHandler);


        // Sensor
        PotentiometerSensor potentiometerSensor = new PotentiometerSensor();
        components.add(potentiometerSensor);
        //telemetryEnabled.add(potentiometerHandler);

        IMUSensor imuSensor = new IMUSensor();
        components.add(imuSensor);


        // Arm
        RotationHandler rotationHandler = new RotationHandler ();
        components.add(rotationHandler);
        //telemetryEnabled.add(rotationHandler);

        ExtensionHandler extensionHandler = new ExtensionHandler();
        components.add(extensionHandler);
        //telemetryEnabled.add(extensionHandler);

        IntakeSpinnerHandler intakeSpinnerHandler = new IntakeSpinnerHandler();
        components.add(intakeSpinnerHandler);


        // Driver
        CollyRandomWheelDriver wheelDriver = new CollyRandomWheelDriver(0, 1, 5, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_Y, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_X, GamePadHandler.GamepadSticks.RIGHT_JOYSTICK_X);
        components.add(wheelDriver);
        telemetryEnabled.add(wheelDriver);

        EACDriver eacDriver = new EACDriver();
        components.add(eacDriver);
        telemetryEnabled.add(eacDriver);


        // Controllers
        CollyWheelController wheelController = new CollyWheelController();
        KarstenEACController eacController = new KarstenEACController();
        eacController.link(gamePadHandler);
        eacController.linkExtensionHandler(extensionHandler);


        // Link the components to each other
        wheelDriver.link(gamePadHandler, wheelHandler, imuSensor, wheelController);
        rotationHandler.link(potentiometerSensor, extensionHandler);
        extensionHandler.link(rotationHandler);
        eacDriver.link(gamePadHandler, rotationHandler, extensionHandler, intakeSpinnerHandler, eacController);
        wheelController.link(gamePadHandler);
    }
}
