package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.component.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.task.AutonTaskRunner;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.Task_DriveToPose;

import java.util.Locale;


@Autonomous(name="Auton_TaskRunnerDrive_Demo", group="Test")
public class Auton_TaskRunnerDrive_Demo extends OpMode {

    final String FRONT_LEFT_DRIVE_MOTOR_NAME = "front_left";
    final String FRONT_RIGHT_DRIVE_MOTOR_NAME = "front_right";
    final String REAR_LEFT_DRIVE_MOTOR_NAME = "rear_left";
    final String REAR_RIGHT_DRIVE_MOTOR_NAME = "rear_right";

    private MecanumDrive drive;
    private GoBildaPinpointDriver pinpoint;

    private Pose2D targetPoseFwd = new Pose2D(DistanceUnit.MM, 500.0, 0.0, AngleUnit.DEGREES, 0.0);
    private Pose2D targetPoseRight = new Pose2D(DistanceUnit.MM, 0.0, 500.0, AngleUnit.DEGREES, 0.0);

    private AutonTaskRunner autonTaskRunner;

    public void init() {

        // INIT DRIVETRAIN
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, FRONT_LEFT_DRIVE_MOTOR_NAME);
        DcMotor frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_DRIVE_MOTOR_NAME);
        DcMotor rearLeft  = hardwareMap.get(DcMotor.class, REAR_LEFT_DRIVE_MOTOR_NAME);
        DcMotor rearRight = hardwareMap.get(DcMotor.class, REAR_RIGHT_DRIVE_MOTOR_NAME);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        drive = new MecanumDrive(frontLeft, frontRight, rearLeft, rearRight);

        // INIT PINPOINT
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        pinpoint.setOffsets(-82.5, 125, DistanceUnit.MM); // TODO: check if signs are correct +/-
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();

        Task_DriveToPose driveTaskFwd = new Task_DriveToPose(
                drive,
                pinpoint,
                targetPoseFwd,
                telemetry
        );
        Task_DriveToPose driveTaskRight = new Task_DriveToPose(
                drive,
                pinpoint,
                targetPoseRight,
                telemetry
        );
        Task[] theTasks = {driveTaskFwd, driveTaskRight};
        autonTaskRunner = new AutonTaskRunner(theTasks);

        // INIT OTHER MECHANISMS - SHOOTER, INTAKE, LIFT, LIMELIGHT, ETC.

        // PRINT TELEMETRY
        Pose2D pos = pinpoint.getPosition();
        String position = String.format(
                Locale.US, "{CurrX: %.3f, CurrY: %.3f, CurrH: %.3f}",
                pos.getX(DistanceUnit.MM),
                pos.getY(DistanceUnit.MM),
                pos.getHeading(AngleUnit.DEGREES)
        );
        telemetry.addData("Position", position);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void loop() {
        autonTaskRunner.execute();
    }
}
