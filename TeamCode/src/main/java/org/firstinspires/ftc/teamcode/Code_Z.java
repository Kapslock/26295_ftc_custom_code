package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.Locale;

@TeleOp(name="Final code", group="Linear OpMode")
public class Code_Z extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriver odo;
    RobotControl robot = new RobotControl(this);
    AutoControl auto = new AutoControl(this);
    private EnhancedNavigation navigation;
    private double mod = 1;
    private double slow = 1;
    private boolean isNavigating = false;
    private boolean bing = false;

    // Basket coordinates (adjust these based on your field setup)
    private final double BASKET_X = robot.BASKET_X_TELE;  // Using constant from RobotControl
    private final double BASKET_Y = robot.BASKET_Y_TELE;  // Using constant from RobotControl
    private static final double BASKET_HEADING = -40.0;
    private double specPhase = 0;

    private double navX = 0;
    private double navY = 0;
    private double navH = 0;// Degrees
    private boolean isEditing = false;
    private boolean nearBasket = false;
    double basketXError = BASKET_X;
    double basketYError = BASKET_Y;
    double basketHError = BASKET_HEADING;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(6.25, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //odo.resetPosAndIMU();

        // Initialize navigation system
        navigation = new EnhancedNavigation(robot, odo);


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Normal teleop control when not navigating to basket
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Apply deadband

            robot.controllerDrive(axial, lateral, yaw, mod);

            // Speed control
            if (gamepad1.right_stick_button) {
                slow = (mod == 1) ? 0.4 : 1;
            }
            if (!gamepad1.right_stick_button) {
                mod = slow;
            }


            // Update odometry and telemetry
            odo.update();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.MM),
                    pos.getY(DistanceUnit.MM),
                    pos.getHeading(AngleUnit.DEGREES));

            // Telemetry updates
            telemetry.addData("odo Position", data);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm motor position", robot.armMotor.getCurrentPosition());
            telemetry.addData("Arm motor target", robot.armTarget);
            telemetry.addData("near basker", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("editing", odo.getHeading());
            telemetry.addData("basket h errpr", basketHError);
            if (isNavigating) {
                telemetry.addData("Navigation", "Moving to Basket");
            }
            telemetry.update();
        }
    }
}