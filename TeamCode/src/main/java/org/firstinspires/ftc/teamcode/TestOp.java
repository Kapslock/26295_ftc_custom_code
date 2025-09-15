package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.Locale;

@TeleOp(name="Final code", group="Linear OpMode")
public class TestOp extends LinearOpMode {
    private LinearOpMode myOpMode = null; // gain access to methods in the calling OpMode.
    private DcMotor skibidi = null;
    private DcMotor skibidii = null;
    private DcMotor skibidiii = null;
    private DcMotor skibidiiii = null;
    @Override
    public void runOpMode() {
        skibidi = myOpMode.hardwareMap.get(
                DcMotor.class,
                "left_front_drive"
        );
        skibidii = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");

        skibidiii = myOpMode.hardwareMap.get(
                DcMotor.class,
                "right_front_drive"
        );
        skibidiiii = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");

        waitForStart();

        skibidi.setPower(1);
        skibidii.setPower(1);
    }
}