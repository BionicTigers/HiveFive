package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Spinner extends Mechanism{

    //Controller buttons
    public boolean spin;
    public boolean spinBack;
    public boolean servoB;

     //Used to declare new instances of Spinner
    public Spinner(CRServo spinner, Servo carouselB) {
        super();
        crServos.add(spinner);
        servos.add(carouselB);
    }

    //Carousel arm methods
     //Moves the carousel arm out
    public void moveArmOut() {
        servos.get(0).setPosition(0.3);
    }

     //Moves the carousel arm back over the robot
    public void moveArmBack(Servo carouselB){
        carouselB.setPosition(0);
    }

    //Carousel spinner methods
     //Spins the carousel spinner for a set amount of time
//    public void spin(int time) {
//        crServos.get(0).setPower(100);
//        try {
//            wait(time*1000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        crServos.get(0).setPower(0);
//    }

    public void update(Gamepad gp1, Gamepad gp2) {
        spinBack = gp2.a;
        servoB = gp2.b;
    }

    public void write() {
        if (spin) {
            crServos.get(0).setPower(100);
            servos.get(0).setPosition(0.5);
        } else if (spinBack){
            crServos.get(0).setPower(-100);
            servos.get(0).setPosition(.2);
        } else if (servoB){
            crServos.get(0).setPower(0);
            servos.get(0).setPosition(.48);
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