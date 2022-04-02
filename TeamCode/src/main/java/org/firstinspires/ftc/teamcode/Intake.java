package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class Intake extends Mechanism {
    Deadline stop = new Deadline(500, TimeUnit.MILLISECONDS);

    //Both of these variables control the direction of the intake
    public boolean goingIn;
    public boolean goingOut;

    /*
     * Creates, declares, and assigns a motor to the motors array list
     */
    public Intake(DcMotorEx intake, Servo intakeServo) {
        super();
        motors.add(intake);
        motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        getServos().add(intakeServo);
    }

    /*
     * Controls the robot during TeleOp and sends input to run
     */
    public void update(Gamepad gp1, Gamepad gp2) {
        goingIn = gp1.left_trigger >= .3;
        goingOut = gp1.right_trigger >= .3 || gp1.x;
        if(gp1.right_trigger >= .3){stop.reset();}
    }

     //Controls the intake
    public void write() {
        run(goingIn, goingOut);

        if(!stop.hasExpired() && !goingIn){
            motors.get(0).setPower(-1);
        }
    }


     //Controls the intake during TeleOp using input from update
    public void run(boolean in, boolean out) {
        if (in) {
            motors.get(0).setPower(1);

        } else if (out) {
            motors.get(0).setPower(-1);
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