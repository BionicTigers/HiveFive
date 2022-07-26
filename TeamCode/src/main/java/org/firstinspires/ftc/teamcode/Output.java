package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class Output extends Mechanism {

    //This variable controls the position of the dropper
    public boolean drop;
    public boolean reset;
    public Servo servo;
    public boolean drop2;
    public boolean mid;
    Deadline open = new Deadline(500, TimeUnit.MILLISECONDS);
     //Creates, declares, and assigns a servo to the servos array list
    public Output(Servo d) {
        super();
        servo = d;
        getServos().add(servo);
    }

    //Deposits an object in the output and returns it to
    public void deposit(){
        open.reset();
        servo.setPosition(0.8);
        if(open.hasExpired()) {
            servo.setPosition(0.2);
        }
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        drop = gp2.dpad_right;
        drop2 = gp2.dpad_left;
        reset = gp2.left_trigger >= 0.2 || gp1.right_trigger >= .2 || gp2.dpad_down;
        if(gp2.left_trigger >= 0.2 || gp1.right_trigger >= .2 || gp2.dpad_down){
            mid = true;
        } else if(gp2.right_trigger>=0.2){
            mid = false;
        }
    }

    public void write() {
        if (drop && !mid) { //If A is being pressed
            servo.setPosition(1); //Move to dropping position
        }
        else if (drop2 && !mid){
            servo.setPosition(.4);
        }
        else if (reset) { //If left trigger is pressed down
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