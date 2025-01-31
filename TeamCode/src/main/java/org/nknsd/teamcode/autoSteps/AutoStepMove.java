package org.nknsd.teamcode.autoSteps;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNAutoStep;
import org.nknsd.teamcode.helperClasses.AutoSkeleton;

public class AutoStepMove extends NKNAutoStep {
    AutoSkeleton autoSkeleton;
    boolean done = false;
    private final double xDist;
    private final double yDist;

    public AutoStepMove(double xDist, double yDist) {
        this.xDist = xDist;
        this.yDist = yDist;
    }

    @Override
    public void link(AutoSkeleton autoSkeleton) {
        this.autoSkeleton = autoSkeleton;
    }

    @Override
    public void run(Telemetry telemetry, ElapsedTime runtime) {
        done = autoSkeleton.runToPosition(telemetry, runtime);
    }

    @Override
    public boolean isDone(ElapsedTime runtime) {
        return done;
    }

    @Override
    public String getName() {
        return "Moving horizontally + " + xDist + " units & vertically " + yDist + " units.";
    }

    @Override
    public void begin(ElapsedTime runtime, Telemetry telemetry) {
        autoSkeleton.setTargetPosition(autoSkeleton.targetPositions[0] + xDist, autoSkeleton.targetPositions[1] + yDist);
    }
}
