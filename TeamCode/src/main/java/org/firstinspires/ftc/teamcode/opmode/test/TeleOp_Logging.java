package org.firstinspires.ftc.teamcode.opmode.test;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.Console;
import java.util.Locale;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

@TeleOp(name="TeleOp_Logging", group="Iterative OpMode")
public class TeleOp_Logging extends OpMode {

    private final Locale locale = Locale.US;
    private final Logger logger = Logger.getLogger("SparkSquadLogger");
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        ConsoleHandler consoleHandler = new ConsoleHandler();
        logger.addHandler(consoleHandler);
        logger.setLevel(Level.ALL);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        logger.info("Elapsed Time:" + runtime.milliseconds());
    }
}
