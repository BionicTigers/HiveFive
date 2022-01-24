package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake extends Mechanism {

    //Both of these variables control the direction of the intake
    public float goingIn;
    public float goingOut;

    /*
     * Creates, declares, and assigns a motor to the motors array list
     */
    public Intake(DcMotorEx intake) {
        super();
        motors.add(intake);
    }

    /*
     * Controls the robot during TeleOp and sends input to run
     */
    public void update(Gamepad gp1, Gamepad gp2) {
        goingIn = gp1.left_trigger;
        goingOut = gp1.right_trigger;
    }

     //Controls the intake
    public void write() {
        run(goingIn, goingOut);
    }

     //Controls the intake during TeleOp using input from update
    public void run(float in, float out) {
        if (in > .25) {
            if(in > .8)
            {
                motors.get(0).setPower(-1);
            }
            else{
                motors.get(0).setPower(0-in);
            }

        } else if (out > .25) {
            if(out > .8)
            {
                motors.get(0).setPower(1);
            }
            else {
                motors.get(0).setPower(out);
            }
        } else {
            motors.get(0).setPower(0);
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