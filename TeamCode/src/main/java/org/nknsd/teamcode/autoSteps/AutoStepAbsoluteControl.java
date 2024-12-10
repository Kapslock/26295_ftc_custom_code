package org.nknsd.teamcode.autoSteps;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNAutoStep;
import org.nknsd.teamcode.helperClasses.AutoSkeleton;

public class AutoStepAbsoluteControl extends NKNAutoStep {
    boolean done = false;
    private final double heading;
    private final double xTarg;
    private final double yTarg;

    public AutoStepAbsoluteControl(double xTarg, double yTarg, double heading) {
        this.heading = heading;
        this.xTarg = xTarg;
        this.yTarg = yTarg;
    }

    @Override
    public void link(AutoSkeleton autoSkeleton) {
        this.autoSkeleton = autoSkeleton;

    }

    public void begin(ElapsedTime runtime, Telemetry telemetry) {
        autoSkeleton.setTargetPosition(xTarg, yTarg);
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
