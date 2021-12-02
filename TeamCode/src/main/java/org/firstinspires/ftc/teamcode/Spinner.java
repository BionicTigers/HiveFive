package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Spinner extends Mechanism{
    
    public boolean spin;
    public boolean spinBack;
    public boolean servoB;
    public Speener(CRServo spinner, Servo carouselB) {
        super();
        crServos.add(spinner);
        servos.add(carouselB);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        spin = gp2.y;
        spinBack = gp2.x;
        servoB = gp2.a;
    }

    public void write() {
        if (spin) {
            crServos.get(0).setPower(60);
        } else if (spinBack){
            crServos.get(0).setPower(-60);
        } else if (servoB){
            crServos.get(0).setPower(0);
            servos.get(0).setPosition(1);
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