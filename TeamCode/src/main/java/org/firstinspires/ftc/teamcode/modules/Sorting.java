package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Sorting {
    private final ElapsedTime timer = new ElapsedTime();
    enum Scan {LEFT, RIGHT, BETWEEN} // расположение зеленого арфтефакта

    private DcMotor drumMotor;
    private Servo wall;
    private Scan pos;
    private Scan pos2;
    public static double SPEED = 0.5;
    public static double OPEN_WALL;
    public static double CLOSE_WALL;

    public static final double PULSES = 537.7;
    boolean green;
    boolean purple;
    boolean a; // 1 датчик
    boolean b;// 2 датчик
    boolean c;// 3 датчик

    public static final double DEGREES = PULSES / 360; // Использование: DEGREES * distanse

    SortMotorDriver sortMotorDriver = new SortMotorDriver();


    public Sorting(LinearOpMode opMode, Scan scan, Scan scan2) {
        this.drumMotor = opMode.hardwareMap.get(DcMotor.class, "drumMotor");
        this.wall = opMode.hardwareMap.get(Servo.class, "wall");
        drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drumMotor.setDirection(DcMotor.Direction.REVERSE);
        this.pos = scan;
        this.pos2 = scan2;
    }

    public class SortMotorDriver extends Thread {

        @Override
        public void run() {
            drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            timer.reset();

            while (!isInterrupted()) {

                getter();
                drumTurner(pos, pos2);
                shooter(drumMotor.getCurrentPosition());


            }
        }
    }

    public void turn() {

    }

    public void getter() {
        int i = 120;
        while (c = !green || !purple) { // 3 датчик нечего не видит
            i += 120;
            while (timer.milliseconds() < 500){}
            if (a = green || purple)// если 1 датчик видит артефакт
                while (drumMotor.getCurrentPosition() <= DEGREES * i) drumMotor.setPower(SPEED);
        }
    }

    public void shooter(int pos) {
        switchWall();
        while (b = green || purple) {// если 2 датчик (у запуска) видит артефакт
            while (drumMotor.getCurrentPosition() <= DEGREES * 120) drumMotor.setPower(SPEED);
            while (timer.milliseconds() < 500){}
        }
    }

    public void switchWall() {
        if (wall.getPosition() == OPEN_WALL) {
            wall.setPosition(CLOSE_WALL);
        } else {
            wall.setPosition(OPEN_WALL);
        }
    }

    public void drumTurner(Scan a, Scan b) {
        switch (a) {
            case LEFT: {
                switch (b) {
                    case LEFT:
                        break;
                    case RIGHT:
                        while (drumMotor.getCurrentPosition() >= DEGREES * 160)
                            drumMotor.setPower(-SPEED);
                        break;
                    case BETWEEN:
                        while (drumMotor.getCurrentPosition() >= DEGREES * 40)
                            drumMotor.setPower(-SPEED);
                        break;
                }
                break;
            }
            case RIGHT: {
                switch (b) {
                    case LEFT:
                        while (drumMotor.getCurrentPosition() >= DEGREES * 40)
                            drumMotor.setPower(-SPEED);
                        break;
                    case RIGHT:
                        break;
                    case BETWEEN:
                        while (drumMotor.getCurrentPosition() >= DEGREES * 160)
                            drumMotor.setPower(-SPEED);
                        break;
                }
                break;
            }
            case BETWEEN: {
                switch (b) {
                    case LEFT:
                        while (drumMotor.getCurrentPosition() >= DEGREES * 40)
                            drumMotor.setPower(-SPEED);
                        break;
                    case RIGHT:
                        while (drumMotor.getCurrentPosition() >= DEGREES * 160)
                            drumMotor.setPower(-SPEED);
                        break;
                    case BETWEEN:
                        break;
                }
                break;
            }

        }
    }


}



