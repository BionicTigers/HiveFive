package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Spinner extends Mechanism {

    //Controller buttons
    public boolean deployed;
    public boolean aIsPressed;
    public boolean spinning;
    public int x = 0;

    //Used to declare new instances of Spinner
    public Spinner(DcMotorEx spinner, Servo carouselB) {
        super();
        motors.add(spinner);
        motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servos.add(carouselB);
        motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Carousel arm methods
    //Moves the carousel arm out
    public void moveArmOut() {
        servos.get(0).setPosition(0.3);
    }

    //Moves the carousel arm back over the robot
    public void moveArmBack(Servo carouselB) {
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
        if(gp2.right_bumper){
            x = x+50;
            motors.get(0).setTargetPosition(x);
            motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors.get(0).setPower(0.35*4/3);
        }
    }

    public void write() {
        if (deployed) {
            servos.get(0).setPosition(0);
        } else {
            servos.get(0).setPosition(0.5);
        }
        if (spinning && motors.get(0).getCurrentPosition() >= 1000) {
            motors.get(0).setPower(0.75*4/3);
        }
        if (motors.get(0).getCurrentPosition() >= 1800) {
            motors.get(0).setPower(0);
            motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            x = 0;
        }
    }

    public void autoSpin() {
        motors.get(0).setTargetPosition(1800);
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.get(0).setVelocity(1600);
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