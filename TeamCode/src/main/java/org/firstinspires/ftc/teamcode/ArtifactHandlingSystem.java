package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Random;

public class ArtifactHandlingSystem {

    private final DcMotor outtakeMotor;
    private final Servo hoodServo;
    private final ColorSensor colorSensor;
    private final Servo rgbIndicator;
    private final DcMotor intakeMotor;
    private final CRServo leftContainerServo;
    private final CRServo rightContainerServo;
    private boolean celebrateOn = false;
    private boolean previousCelebrate = false;
    private final LinearOpMode linearOpMode;

    public ArtifactHandlingSystem(LinearOpMode linearOpMode) {
        this.outtakeMotor = linearOpMode.hardwareMap.dcMotor.get("outtakeMotor");
        this.hoodServo = linearOpMode.hardwareMap.servo.get("hoodServo");
        this.intakeMotor = linearOpMode.hardwareMap.dcMotor.get("intakeMotor");
        this.colorSensor = linearOpMode.hardwareMap.colorSensor.get("colorSensor");
        this.rgbIndicator = linearOpMode.hardwareMap.servo.get("rgbIndicator");
        this.leftContainerServo = linearOpMode.hardwareMap.crservo.get("leftContainerServo");
        this.rightContainerServo = linearOpMode.hardwareMap.crservo.get("rightContainerServo");
        this.linearOpMode = linearOpMode;
    }

    public void configureMotorModes() {
        outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftContainerServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightContainerServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void shootArtifact(float shootArtifact) {
        outtakeMotor.setPower(shootArtifact);
    }

    public void adjustHoodAngle(boolean increaseAngle, boolean decreaseAngle) {
        double position = hoodServo.getPosition();
        double newPosition;

        if (increaseAngle) {
            newPosition = position + 1;
        } else if (decreaseAngle) {
            newPosition = position - 1;
        } else {
            return;
        }

        hoodServo.setPosition(Math.max(ConstantsTeleOp.SHOOTING_ANGLE_MIN, Math.min(ConstantsTeleOp.SHOOTING_ANGLE_MAX, newPosition)));
    }

    public void intakeArtifact(boolean intakeArtifact) {
        if (intakeArtifact) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }
    }

    public void handleCelebrateToggle(boolean celebrate) {
        if (celebrate && !previousCelebrate) {
            celebrateOn = !celebrateOn;
        }

        previousCelebrate = celebrate;

        if (celebrateOn) {
            Random random = new Random();
            double randomColor = ConstantsTeleOp.RED + (ConstantsTeleOp.VIOLET - ConstantsTeleOp.RED) * random.nextDouble();
            rgbIndicator.setPosition(randomColor);

            // Wait for 250 milliseconds
            linearOpMode.sleep(250);
        }
    }

    private Object[] detectColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        float hue = hsv[0];
        float value = hsv[2];

        String color;

        if ((hue > 260) && (hue < 300)) {
            color = "Purple";
        } else if ((hue > 90) && (hue < 160)) {
            color = "Green";
        } else if (value < 0.15) {
            color = "Black";
        } else {
            color = "Unknown";
        }

        return new Object[] {color, hue};
    }

    public void manuallySendArtifact(boolean manuallySend) {
        if (manuallySend) {
            leftContainerServo.setPower(1);
            rightContainerServo.setPower(1);
        } else {
            leftContainerServo.setPower(0);
            rightContainerServo.setPower(0);
        }
    }

    public void sendArtifactToOuttake() {
        if (celebrateOn) {
            return; // Don't process artifacts while celebrating
        }

        Object[] detectedColor = detectColor();
        String color = (String) detectedColor[0];
        setRGBIndicator(color);

        switch (color) {
            case "Purple":
            case "Green":
                leftContainerServo.setPower(0);
                rightContainerServo.setPower(0);
                break;
            case "Black":
            case "Unknown":
                leftContainerServo.setPower(1);
                rightContainerServo.setPower(1);
                break;
        }
    }

    public void shootAutoArtifact(){
        Thread ArtifactShoot = new Thread(() -> {
            long endTime = System.currentTimeMillis() + 4000; // 4 seconds from now
            
            // Start the outtake motor for shooting
            shootArtifact(0.8F);
            
            while (System.currentTimeMillis() < endTime) {

                sendArtifactToOuttake();

                }

            

            shootArtifact(0.0F);
            System.out.println("Thread finished its 4-second run!");
        });
        
        // Start the thread
        ArtifactShoot.start();
    }

    private void setRGBIndicator(String color) {
        switch (color) {
            case "Purple":
                rgbIndicator.setPosition(ConstantsTeleOp.PURPLE);
                break;
            case "Yellow":
                rgbIndicator.setPosition(ConstantsTeleOp.YELLOW);
                break;
            case "Black":
                rgbIndicator.setPosition(ConstantsTeleOp.BLACK);
                break;
            case "Unknown":
                rgbIndicator.setPosition(ConstantsTeleOp.UNKNOWN);
                break;
        }
    }

    private double getRGBIndicatorPosition() {
        return rgbIndicator.getPosition();
    }

    private boolean isCelebrateOn() {
        return celebrateOn;
    }

    public void displayTelemetry() {
        linearOpMode.telemetry.addData("Outtake Motor Power", outtakeMotor.getPower());
        linearOpMode.telemetry.addData("Intake Motor Power", intakeMotor.getPower());
        linearOpMode.telemetry.addData("Detected Color", detectColor()[0]);
        linearOpMode.telemetry.addData("Hue", detectColor()[1]);
        linearOpMode.telemetry.addData("RGB Position", getRGBIndicatorPosition());
        linearOpMode.telemetry.addData("Celebrate Mode", isCelebrateOn());
    }
}
