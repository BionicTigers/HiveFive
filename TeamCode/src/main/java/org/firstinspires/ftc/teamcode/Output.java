package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Output extends Mechanism{

    //Both of these variables control the position of the dropper
    public boolean drop;
    public boolean raise;

    //Creates, declares, and assigns a servo to the servos array list
    public Output (Servo dropper) {
        super();
        getServos().add(dropper);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp2.a) { //If A is pressed
            drop = true; //Set mode: Drop
        }
        if (gp2.y) { //If Y is pressed
            raise = true; //Set mode: Raise
        }
    }

    public void write() {
        if (drop) { //If mode is Drop
            servos.get(0).setPosition(0.2); //Move to dropping position
        } else if (raise) { //If mode is Raise
            servos.get(0).setPosition(0.8); //Move to raised position
        }
    }
}

/*
    Pseudocode:
    When the drop button is pressed
    Drop the payload
    Close the trapdoor again
*/