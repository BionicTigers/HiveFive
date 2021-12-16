package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Output extends Mechanism {

    //This variable controls the position of the dropper
    public boolean drop;

    //Creates, declares, and assigns a servo to the servos array list
    public Output(Servo dropper) {
        super();
        getServos().add(dropper);
    }

    /**
     * Deposits an object in the output
     * @param dropper   servo that controls the output
     */
    public void deposit(Servo dropper){
        dropper.setPosition(0.2);
        try {
            wait(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        dropper.setPosition(0.8);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        drop = gp2.a;
    }

    public void write() {
        if (drop) { //If A is being pressed
            servos.get(0).setPosition(0.15); //Move to dropping position
        } else { //If A isn't being pressed

            servos.get(0).setPosition(1); //Move to upright position
        }
    }
}

/*
    Pseudocode:
    When the drop button is pressed
    Drop the payload
    Close the trapdoor again
*/