package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake extends Mechanism {

    //Both of these variables control the direction of the intake
    public boolean goingIn;
    public boolean goingOut;

    //Both of these variables control the position of the intake
    public boolean up;
    public boolean down;

    //Creates, declares, and assigns a motor to the motors array list
    public Intake(DcMotorEx intakeMotor, Servo intakeLeft, Servo intakeRight) {
        super();
        motors.add(intakeMotor);
        servos.add(intakeLeft);
        servos.add(intakeRight);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp1.right_trigger >= 0.5) { //If right trigger is pressed
            goingIn = true; //Set direction to in
        } else if (gp1.left_trigger >= 0.5) { //Otherwise if left trigger is pressed
            goingOut = true; //Set direction to out
        } else { //If neither is pressed
            //Set direction to none
            goingIn = false;
            goingOut = false;
            //-----
        }

        if (gp1.right_bumper){
            up = true;
        } else if (gp1.left_bumper){
            down = true;
        } else{
            up = false;
            down = false;
        }
    }


    public void write() {
        if (goingIn) { //If direction is set to in
            runIn(); //Set intake direction to in
        } else if (goingOut) { //If direction is set to out
            runOut(); //Set intake direction to out
        } else{ //If neither direction is chosen
            stop(); //Stop motor
        }

        if(up){
            servos.get(0).setPosition(1);
            servos.get(1).setPosition(0.8);
        } else if(down){
            servos.get(0).setPosition(0.8);
            servos.get(1).setPosition(1);
        }
    }

    public void runIn() {
        motors.get(0).setPower(-100);
    }

    public void runOut() {
        motors.get(0).setPower(100);
    }

    public void stop() {
        motors.get(0).setPower(0);
    }
}

/*
    Pseudocode:
    When a button is pressed
    Turn of intake
    When another button is pressed
    Reverse intake
    (Optionally) Reverse intake when there is more than one cube in the robot
*/