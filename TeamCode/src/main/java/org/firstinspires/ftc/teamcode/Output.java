package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Output extends Mechanism {

    //This variable controls the position of the dropper
    public boolean drop;
    public double transferDown;

    //Creates, declares, and assigns a servo to the servos array list
    public Output(Servo dropper) {
        super();
        getServos().add(dropper);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        drop = gp2.a;
        transferDown = gp2.left_trigger;
    }

    public void write() {
        if (drop) { //If A is being pressed
            servos.get(0).setPosition(0); //Move to dropping position
        } else { //If A isn't being pressed
            servos.get(0).setPosition(1); //Move to upright position
        }
        if(transferDown >= 0.4){
            servos.get(0).setPosition(0.7);
        }
    }
}

/*
    Pseudocode:
    When the drop button is pressed
    Drop the payload
    Close the trapdoor again
*/