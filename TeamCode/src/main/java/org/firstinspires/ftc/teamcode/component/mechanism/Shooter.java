package org.firstinspires.ftc.teamcode.component.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    private final double TRIGGER_INCREMENT = 0.5;
    private final DcMotor shooterMotor;
    private final Servo trigger;

    public Shooter(DcMotor shooterMotor, Servo trigger) {
        this.shooterMotor = shooterMotor;
        this.trigger = trigger;
    }

    public void spinUp(double speed) {
        shooterMotor.setPower(speed);
    }

    public void shoot() {
        double triggerPosition = trigger.getPosition();
        trigger.setPosition(triggerPosition + TRIGGER_INCREMENT);
    }
}
