package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


public class Sorting {
    private DcMotor drum;
    private Servo wall;

    public static double SPEED;
    public static double OPEN_WALL;
    public static double CLOSE_WALL;


    public void runOpMode(LinearOpMode opMode) {
        this.drum = opMode.hardwareMap.get(DcMotor.class, "drum");
        this.wall = opMode.hardwareMap.get(Servo.class, "wall");
        drum.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drum.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drum.setDirection(DcMotor.Direction.REVERSE);
    }

    public void turnDrum() {

    }

    public void switchWall() {
        if (wall.getPosition() == OPEN_WALL) {
            wall.setPosition(CLOSE_WALL);
        } else {
            wall.setPosition(OPEN_WALL);
        }
    }


}


