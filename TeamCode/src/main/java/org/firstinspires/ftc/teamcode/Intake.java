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
    public boolean mid;
    DcMotorEx intakeMotor;
    Servo intakeLeft;
    Servo intakeRight;

    //Servos
    public Servo blocker;

    /**
     * Creates, declares, and assigns a motor to the motors array list
     */
    public Intake(DcMotorEx intakeM, Servo intakeL, Servo intakeR, Servo block) {
        super();
        motors.add(intakeM);
        intakeMotor = intakeM;
        servos.add(intakeL);
        intakeLeft = intakeL;
        servos.add(intakeR);
        intakeRight = intakeR;
        servos.add(block);
        blocker = block;
    }

    /**
     * Controls the robot during TeleOp and sends input to run
     * @param gp1 the first gamepad
     * @param gp2 the second gamepad
     */
    public void update(Gamepad gp1, Gamepad gp2) {
        goingIn = gp1.right_trigger;
        goingOut = gp1.left_trigger;
        if (gp1.right_bumper) {
            up = true;
        } else if (gp1.left_bumper) {
            down = true;
        } else if (gp1.x) {
            mid = true;
        } else {
            up = false;
            down = false;
            mid = false;
        }
    }

    /**
     * Controls the intake
     */
    public void write() {
        //If direction is set to in
        run(goingIn, goingOut); //Set intake direction to in
        //else if () { //If direction is set to out
//            runOut(); //Set intake direction to out
//        } else{ //If neither direction is chosen
//            stop(); //Stop motor
//        }

        if (up) {
            servos.get(2).setPosition(0.5);
            servos.get(0).setPosition(0.83);
            servos.get(1).setPosition(0.7);
        } else if (mid) {
            servos.get(0).setPosition(0.645);
            servos.get(1).setPosition(0.82);
        } else if (down) {
            servos.get(2).setPosition(1);
            servos.get(0).setPosition(0.54);
            servos.get(1).setPosition(.86);
        }
    }

    /**
     * Controls the intake during TeleOp using input from update
     * @param in
     * @param out
     */
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

//New Intake