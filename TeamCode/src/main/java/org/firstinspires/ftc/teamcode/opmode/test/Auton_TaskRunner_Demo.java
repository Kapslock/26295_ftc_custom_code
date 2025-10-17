package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.task.AutonTaskRunner;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.Task_ElapsedTimeDemo;

@Autonomous(name="Auton_TaskRunner_Demo", group="Test")
public class Auton_TaskRunner_Demo extends OpMode {

    private AutonTaskRunner autonTaskRunner;

    public void init() {
        Task_ElapsedTimeDemo timerTask1 = new Task_ElapsedTimeDemo(1.0f);
        Task_ElapsedTimeDemo timerTask2 = new Task_ElapsedTimeDemo(2.5f);
        Task[] theTasks = {timerTask1, timerTask2};
        autonTaskRunner = new AutonTaskRunner(theTasks);
    }

    public void loop() {
        autonTaskRunner.execute();
    }
}
