package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;


public class Sorting {


    private final ElapsedTime timer = new ElapsedTime();
    private final NormalizedColorSensor colorSensor;
    private final NormalizedColorSensor colorSensor2;
    private final NormalizedColorSensor colorSensor3;

    enum Scan {LEFT, RIGHT, BETWEEN} // расположение зеленого арфтефакта
    //private Scan position;



    enum Color {GREEN, PURPLE, NONE}


    private final DcMotor drumMotor;
    private final Servo wall;
    public Scan pos;
    public static double SPEED = 0.5;
    public static double OPEN_WALL;
    public static double CLOSE_WALL;
    public static final double GREEN_MAX = 100;
    public static final double GREEN_MIN = 100;
    //public static final double PURPLE_MAX = 100;
    //public static final double PURPLE_MIN = 100;


    public static final double PULSES = 537.7;
    public static final float GAIN = 0.2F;
    public static float[] hsv = new float[3];
    public static float[] hsv2 = new float[3];
    public static float[] hsv3 = new float[3];
    boolean green;
    boolean purple;
    boolean a; // 1 датчик
    boolean b;// 2 датчик
    boolean c;// 3 датчик

    public static final double DEGREES = PULSES / 360; // Использование: DEGREES * distanse

    SortMotorDriver sortMotorDriver = new SortMotorDriver();


    public Sorting(LinearOpMode opMode, Scan scan) {
        this.drumMotor = opMode.hardwareMap.get(DcMotor.class, "drumMotor");
        this.wall = opMode.hardwareMap.get(Servo.class, "wall");
        drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drumMotor.setDirection(DcMotor.Direction.REVERSE);
        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor1");
        colorSensor.setGain(GAIN);
        colorSensor2 = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor2");
        colorSensor2.setGain(GAIN);
        colorSensor3 = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor3");
        colorSensor3.setGain(GAIN);
        this.pos = scan;
    }

    public class SortMotorDriver extends Thread {

        @Override
        public void run() {
            drumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drumMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            timer.reset();

            while (!isInterrupted()) {

                intaker();
                drumTurner(pos, artefact_pos(getColor()));
                shooter(drumMotor.getCurrentPosition());


            }
        }
    }
    public void intaker() {
        int i = 120;
        while (getColor().get(2) == Color.NONE) { // 3 датчик нечего не видит
            i += 120;

            if (getColor().get(0) == Color.PURPLE || getColor().get(0) == Color.GREEN) {// если 1 датчик видит артефакт
                while (timer.milliseconds() < 500) {
                }
                while (drumMotor.getCurrentPosition() <= DEGREES * i) drumMotor.setPower(SPEED);
            }
            timer.reset();
        }
    }

    public void shooter(int pos) {
        switchWall();
        while (getColor().get(1) == Color.PURPLE || getColor().get(1) == Color.GREEN) {// если 2 датчик (у запуска) видит артефакт
            while (drumMotor.getCurrentPosition() <= DEGREES * 120) drumMotor.setPower(SPEED);
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

    public ArrayList<Color> getColor() {
        ArrayList<Color> colorSensors = new ArrayList<>();

        NormalizedRGBA color1 = colorSensor.getNormalizedColors();
        NormalizedRGBA color2 = colorSensor2.getNormalizedColors();
        NormalizedRGBA color3 = colorSensor3.getNormalizedColors();
        android.graphics.Color.colorToHSV(color1.toColor(), hsv);
        android.graphics.Color.colorToHSV(color2.toColor(), hsv2);
        android.graphics.Color.colorToHSV(color3.toColor(), hsv3);

        if (hsv[1] < 0.5) {
            if (hsv[0] <= GREEN_MAX && hsv[0] >= GREEN_MIN) colorSensors.add(Color.GREEN);
            else colorSensors.add(Color.PURPLE);
        } else colorSensors.add(Color.NONE);

        if (hsv2[1] < 0.5) {
            if (hsv2[0] <= GREEN_MAX && hsv2[0] >= GREEN_MIN) colorSensors.add(Color.GREEN);
            else colorSensors.add(Color.PURPLE);
        } else colorSensors.add(Color.NONE);

        if (hsv3[1] < 0.5) {
            if (hsv3[0] <= GREEN_MAX && hsv3[0] >= GREEN_MIN) colorSensors.add(Color.GREEN);
            else colorSensors.add(Color.PURPLE);
        } else colorSensors.add(Color.NONE);

        return colorSensors;

    }
    public Scan artefact_pos(ArrayList<Color> a){
        Scan position = null;
        if(a.get(1) == Color.GREEN) position = Scan.LEFT;
        else if(a.get(0) == Color.GREEN) position = Scan.BETWEEN;
        else if(a.get(2) == Color.GREEN) position = Scan.RIGHT;

        return position;
    }


}



