package org.firstinspires.ftc.teamcodealpha.drive.config;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcodealpha.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    /*
     * FTC 17240 GrantBot: (2024-12-26)
     *  Switched from Custom Odometry Pods to GoBuila 4-Bar Odometry Pod (32mm) : https://www.gobilda.com/4-bar-odometry-pod-32mm-wheel/
     *  TODO: Measure LATERAL_DISTANCE and FORWARD_OFFSET
     *  TODO: Set new X_MULTIPLIER and Y_MULTIPLIER (https://learnroadrunner.com/dead-wheels.html#tuning-three-wheel)
     */

    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = .63; // // Dual Omni 32mm (1.2598 inches / 2)
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 7.96; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -1; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private List<Integer> lastEncPositions, lastEncVels;

    /* FTC 17240 2023-24: Duluth Bot + 3 Wheel Odometry
    * Added X_MULTIPLIER and Y_MULTIPLIER */
    public static double X_MULTIPLIER = 0.997; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRearDrive"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRearDrive"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFrontDrive"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        rightEncoder.setDirection(Encoder.Direction.FORWARD);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.FORWARD);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        /* FTC 17240 2023-24: Duluth Bot + 3 Wheel Odometry
         * Added X_MULTIPLIER and Y_MULTIPLIER */
        return Arrays.asList(
                encoderTicksToInches(leftPos) * X_MULTIPLIER,
                encoderTicksToInches(rightPos) * X_MULTIPLIER,
                encoderTicksToInches(frontPos) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        /* FTC 17240 2023-24: Duluth Bot + 3 Wheel Odometry
         * Added X_MULTIPLIER and Y_MULTIPLIER */
        return Arrays.asList(
                encoderTicksToInches(leftVel) * X_MULTIPLIER,
                encoderTicksToInches(rightVel) * X_MULTIPLIER,
                encoderTicksToInches(frontVel) * Y_MULTIPLIER
        );
    }
}
