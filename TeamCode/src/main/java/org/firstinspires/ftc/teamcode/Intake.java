package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake extends Mechanism {

    //Both of these variables control the direction of the intake
    public boolean goingUp;
    public boolean goingDown;

    //Creates, declares, and assigns a motor to the motors array list
    public Intake(Servo intakeLeft, Servo intakeRight) {
        super();
        servos.add(intakeLeft);
        servos.add(intakeRight);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp1.right_trigger >= 0.5) { //If right trigger is pressed
            goingUp = true; //Set direction to in
        } else if (gp1.left_trigger >= 0.5) { //Otherwise if left trigger is pressed
            goingDown = true; //Set direction to out
        } else { //If neither is pressed
            //Set direction to none
            goingUp = false;
            goingDown = false;
            //-----
        }
    }


    public void write() {
        if (goingUp) { //If direction is set to in
            up(); //Move servos to the up position
        } else if (goingDown) { //If direction is set to out
            down(); //Move servos to the down position
        }
    }

    public void up() {
        servos.get(0).setPosition(0.5);
        servos.get(1).setPosition(0.5);
    }

    public void down() {
        servos.get(0).setPosition(0);
        servos.get(1).setPosition(0);
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