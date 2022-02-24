package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Spinner extends Mechanism{

    //Controller buttons
    public boolean deployed;
    public boolean aIsPressed;
    public boolean spinning;

     //Used to declare new instances of Spinner
    public Spinner(DcMotorEx spinner, Servo carouselB) {
        super();
        motors.add(spinner);
        motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp2.a && !aIsPressed && !deployed) {
            aIsPressed = true;
            deployed = true;
        }
        if (!gp2.a && aIsPressed) {
            aIsPressed = false;
        }
        if (deployed && !aIsPressed && gp2.a) {
            autoSpin();
            aIsPressed = true;
        }
        if (gp2.b) {
            deployed = false;
        }
    }

    public void write() {
        if (deployed) {
            servos.get(0).setPosition(0.2);
        } else {
            servos.get(0).setPosition(0.48);
        }
        if (spinning && motors.get(0).getCurrentPosition() >= 1115) {
            motors.get(0).setPower(1);
        }
        if (motors.get(0).getCurrentPosition() >= 2100) {
            motors.get(0).setPower(0);
            motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void autoSpin() {
        motors.get(0).setTargetPosition(2100);
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.get(0).setPower(0.32275);
        spinning = true;
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