package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Spinner extends Mechanism{

    public boolean spin;
    public boolean spinBack;
    public boolean servoB;
    public Spinner(CRServo spinner, Servo carouselB) {
        super();
        crServos.add(spinner);
        servos.add(carouselB);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        spin = gp2.dpad_right;
        spinBack = gp2.dpad_left;
        servoB = gp2.a;
    }

    public void write() {
        if (spin) {
            crServos.get(0).setPower(60);
            servos.get(0).setPosition(1);
        } else if (spinBack){
            crServos.get(0).setPower(-60);
            servos.get(0).setPosition(.5);
        } else if (servoB){
            crServos.get(0).setPower(0);
            servos.get(0).setPosition(0);
        } else{
            crServos.get(0).setPower(0);
        }
    }
}

/*
    Pseudocode:
    (Potentially) Move automatically to the corner
    DIRECTIVE: Spin
    Ducky Profit
    Stop so duck resupply can occur
    Repeat last few steps
*/