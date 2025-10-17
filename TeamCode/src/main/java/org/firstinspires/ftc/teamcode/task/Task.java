package org.firstinspires.ftc.teamcode.task;

public interface Task {

    /**
     * executes this task
     * @return true if task should keep running,  false if it is done
     */
    public boolean execute();
}
