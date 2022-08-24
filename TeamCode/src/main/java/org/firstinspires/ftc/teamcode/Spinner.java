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
    public boolean bIsPressed;
    public boolean spinning;
    public boolean backspinning;
    public int x = 0;

    //Used to declare new instances of Spinner
    public Spinner(DcMotorEx spinner) {
        super();
        motors.add(spinner);
        motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        if (!gp2.a && aIsPressed) {
            aIsPressed = false;
        }
        if (!aIsPressed && gp2.a) {
            autoSpin();
            aIsPressed = true;
        }
        if (!gp2.b && bIsPressed) {
            bIsPressed = false;
        }
        if (!bIsPressed && gp2.b) {
            backSpin();
            bIsPressed = true;
        }
        if(gp2.right_bumper){
            x = x+50;
            motors.get(0).setTargetPosition(x);
            motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors.get(0).setPower(0.35);
        }
    }

    public void write() {
        if (spinning && motors.get(0).getCurrentPosition() <= -1800) {
            motors.get(0).setPower(0.75);
        }
        if (motors.get(0).getCurrentPosition() <= -2400) {
            motors.get(0).setPower(0);
            motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            x = 0;
            spinning = false;
        }
        if (backspinning && motors.get(0).getCurrentPosition() >= 1800) {
            motors.get(0).setPower(0.75);
        }
        if (motors.get(0).getCurrentPosition() >= 2400) {
            motors.get(0).setPower(0);
            motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            x = 0;
            backspinning = false;
        }
    }

    public void autoSpin() {
        motors.get(0).setTargetPosition(-2400);
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.get(0).setVelocity(1100);
        spinning = true;

    }

    public void backSpin() {
        motors.get(0).setTargetPosition(2400);
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.get(0).setVelocity(1100);
        backspinning = true;

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