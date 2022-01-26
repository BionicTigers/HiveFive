package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Output extends Mechanism {

    //This variable controls the position of the dropper
    public boolean drop;
    public boolean reset;
    public Servo servo;

     //Creates, declares, and assigns a servo to the servos array list
    public Output(Servo d) {
        super();
        servo = d;
        getServos().add(servo);
    }


     //Deposits an object in the output and returns it to
    public void deposit(){
        //It looks ugly, but this is how wait works
        servo.setPosition(0.8);
        try {
            wait(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        servo.setPosition(0.2);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        drop = gp2.a;
        reset = gp2.left_trigger >= 0.2 || gp1.right_trigger >= .2;
    }

    public void write() {
        if (drop) { //If A is being pressed
            servo.setPosition(0.3); //Move to dropping position
        } else if (reset) { //If left trigger is pressed down
            servo.setPosition(.7); //Move to upright position
        }
    }
}

/*
    Pseudocode:
    When the drop button is pressed
    Drop the payload
    Close the trapdoor again
*/