package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RobotControls {
    private final LinearOpMode linearOpMode;

    public RobotControls(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    boolean intakeArtifact;
    boolean rejectIntakeArtifact;
    float shootArtifact;
    float rejectShootingArtifact;
    boolean shiftArtifactInContainer;
    boolean rejectContainerArtifact;
    public void updateControls() {
        intakeArtifact = linearOpMode.gamepad2.a;
        shiftArtifactInContainer = linearOpMode.gamepad2.b;
        rejectContainerArtifact = linearOpMode.gamepad2.x;
        rejectIntakeArtifact = linearOpMode.gamepad2.y;
        rejectShootingArtifact = linearOpMode.gamepad2.left_trigger;
        shootArtifact = linearOpMode.gamepad2.right_trigger;
    }
}
