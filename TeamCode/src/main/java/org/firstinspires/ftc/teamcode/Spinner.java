package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Spinner extends Mechanism {


    //Controller buttons
    public boolean deployed;
    public boolean aIsPressed;
    public boolean spinning;
    public boolean spinningBack;
    public int x = 0;

    //Used to declare new instances of Spinner
    public Spinner(DcMotorEx spinner) {
        super();
        motors.add(spinner);
        motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp2.right_trigger > .2 && !aIsPressed && !deployed) {
            aIsPressed = true;
            deployed = true;
        }
        if (!(gp2.right_trigger > .2) && aIsPressed) {
            aIsPressed = false;
        }
        if (deployed && !aIsPressed && gp2.right_trigger > .2 && gp2.back) {
            autoSpinBack();
            aIsPressed = true;
        } else if (deployed && !aIsPressed && gp2.right_trigger > .2) {
            autoSpin();
            aIsPressed = true;
        }
        if (gp2.b) {
            deployed = false;
        }
        if(gp2.left_trigger > .2){
            x = x+50;
            motors.get(0).setTargetPosition(x);
            motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors.get(0).setPower(0.35);
        }
    }

    public void write() {
        if (spinning && motors.get(0).getCurrentPosition() >= 1000*4/3) {
            motors.get(0).setPower(0.75);
        }

        if (spinningBack && motors.get(0).getCurrentPosition() <= -1000*4/3) {
            motors.get(0).setPower(0.75);
        }

    }

    public void autoSpin() {
        motors.get(0).setTargetPosition(motors.get(0).getCurrentPosition() + (1900));
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.get(0).setVelocity(1500);
        spinning = true;

    }

    public void autoSpinBack() {
        motors.get(0).setTargetPosition(motors.get(0).getCurrentPosition() + (-1900));
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.get(0).setVelocity(1500);
        spinningBack = true;

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