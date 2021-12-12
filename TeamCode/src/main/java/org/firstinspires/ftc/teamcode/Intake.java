package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake extends Mechanism {

    //Both of these variables control the direction of the intake
    public float goingIn;
    public float goingOut;

    //Both of these variables control the position of the intake
    public boolean up;
    public boolean down = true;

    //Creates, declares, and assigns a motor to the motors array list
    public Intake(DcMotorEx intakeMotor, Servo intakeLeft, Servo intakeRight) {
        super();
        motors.add(intakeMotor);
        servos.add(intakeLeft);
        servos.add(intakeRight);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
//        if (gp1.right_trigger >= 0.5) { //If right trigger is pressed
//            goingIn = true; //Set direction to in
//        } else if (gp1.left_trigger >= 0.5) { //Otherwise if left trigger is pressed
//            goingOut = gp1.left_trigger //Set direction to out
//        } else { //If neither is pressed
//            //Set direction to none
        goingIn = gp1.right_trigger;
        goingOut = gp1.left_trigger;
        //-----
        //  }
        if (gp1.right_bumper) {
            up = true;
        } else if (gp1.left_bumper) {
            down = true;
        } else {
            up = false;
            down = false;
        }
    }


    public void write() {
        //If direction is set to in
        run(goingIn, goingOut); //Set intake direction to in
        //else if () { //If direction is set to out
//            runOut(); //Set intake direction to out
//        } else{ //If neither direction is chosen
//            stop(); //Stop motor
//        }

        if (up) {
            servos.get(0).setPosition(0.83);
            servos.get(1).setPosition(0.7);
        } else if (down) {
            servos.get(0).setPosition(0.46);
            servos.get(1).setPosition(.94);
        }
    }

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
//    public void runOut() {
//        motors.get(0).setPower(100);
//    }
//
//    public void stop() {
//        motors.get(0).setPower(0);
//    }
//}

/*
    Pseudocode:
    When a button is pressed
    Turn of intake
    When another button is pressed
    Reverse intake
    (Optionally) Reverse intake when there is more than one cube in the robot
*/