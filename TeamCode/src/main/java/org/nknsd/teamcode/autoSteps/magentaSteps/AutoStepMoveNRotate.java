package org.nknsd.teamcode.autoSteps.magentaSteps;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNAutoStep;
import org.nknsd.teamcode.helperClasses.AutoSkeleton;

public class AutoStepMoveNRotate extends NKNAutoStep {
    AutoSkeleton autoSkeleton;
    boolean done = false;
    private final double heading;
    private final double xDist;
    private final double yDist;

    public AutoStepMoveNRotate(double xDist, double yDist, double heading) {
        this.heading = heading;
        this.xDist = xDist;
        this.yDist = yDist;
    }

    @Override
    public void link(AutoSkeleton autoSkeleton) {
        this.autoSkeleton = autoSkeleton;

    }

    public void begin(ElapsedTime runtime, Telemetry telemetry) {
        autoSkeleton.setTargetPosition(autoSkeleton.targetPositions[0] + xDist, autoSkeleton.targetPositions[1] + yDist);
        autoSkeleton.setTargetRotation(heading);
    }

    @Override
    public void run(Telemetry telemetry, ElapsedTime runtime) {
        done = autoSkeleton.runToPosition(telemetry, runtime);
    }

    @Override
    public boolean isDone(ElapsedTime runtime) {return done;}

    @Override
    public String getName() {
        return "Rotating to " + heading;
    }
}
