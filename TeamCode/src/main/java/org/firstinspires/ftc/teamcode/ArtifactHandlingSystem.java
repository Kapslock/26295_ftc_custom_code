package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ArtifactHandlingSystem {

    private final DcMotor outtakeMotor;
    private final DcMotor intakeMotor;
    private final DcMotor leftContainerMotor;
    private final DcMotor rightContainerMotor;
    private final LinearOpMode linearOpMode;

    public ArtifactHandlingSystem(LinearOpMode linearOpMode) {
        this.outtakeMotor = linearOpMode.hardwareMap.dcMotor.get("outtakeMotor");
        this.intakeMotor = linearOpMode.hardwareMap.dcMotor.get("intakeMotor");
        this.leftContainerMotor = linearOpMode.hardwareMap.dcMotor.get("leftContainerMotor");
        this.rightContainerMotor = linearOpMode.hardwareMap.dcMotor.get("rightContainerMotor");
        this.linearOpMode = linearOpMode;
    }

    public void configureMotorModes() {
        outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftContainerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightContainerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intakeSystem(boolean intakeArtifact, boolean rejectArtifact) {
        if (intakeArtifact) {
            intakeMotor.setPower(1);
        } else if (rejectArtifact) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }
    }

    public void containerSystem(boolean sendArtifact, boolean rejectArtifact) {
        if (sendArtifact) {
            leftContainerMotor.setPower(0.5);
            rightContainerMotor.setPower(0.5);
        } else if (rejectArtifact) {
            leftContainerMotor.setPower(-0.5);
            rightContainerMotor.setPower(-0.5);
        } else {
            leftContainerMotor.setPower(0);
            rightContainerMotor.setPower(0);
        }
    }

    public void shootingSystem(float shootArtifact, float rejectArtifact) {
        if (shootArtifact > 0) {
            outtakeMotor.setPower(shootArtifact);
        } else if (rejectArtifact > 0) {
            outtakeMotor.setPower(-rejectArtifact);
        } else {
            outtakeMotor.setPower(0);
        }
    }

    public void displayTelemetry() {
        linearOpMode.telemetry.addData("Outtake Motor Power", outtakeMotor.getPower());
        linearOpMode.telemetry.addData("Intake Motor Power", intakeMotor.getPower());
        linearOpMode.telemetry.addData("Left Container Power", leftContainerMotor.getPower());
        linearOpMode.telemetry.addData("Right Container Power", rightContainerMotor.getPower());
    }
}
