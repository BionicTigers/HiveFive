package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake extends Mechanism {

    //Both of these variables control the direction of the intake
    public boolean goingin;
    public boolean goingout;

    //Creates, declares, and assigns a motor to the motors array list
    public Intake (DcMotorEx intaker) {
        super();
        motors.add(intaker);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp1.right_trigger >= 0.5) { //If right trigger is pressed
            goingin = true; //Set direction: In
        } else {
            goingin = false;
        }
        if (gp1.left_trigger >= 0.5) { //Otherwise if left trigger is pressed
            goingout = true; //Set direction: Out
        } else {
            goingout = false;
        }
    }

    public void write() {
        if (goingin) { //If direction is In
            motors.get(0).setPower(1); //Turn on the intake
        } else if (goingout) { //If direction is Out
            motors.get(0).setPower(-1); //Reverse the intake
        } else { //If no direction is set
            motors.get(0).setPower(0); //Stop the intake
        }
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