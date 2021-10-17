package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake extends Mechanism {

    public boolean goingin;
    public boolean goingout;

    public Intake (DcMotorEx intaker) {
        super();
        motors.add(intaker);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp1.right_trigger >= 0.5) {
            goingin = true;
        } else if (gp1.left_trigger >= 0.5) {
            goingout = true;
        }
    }

    public void write() {
        if (goingin) {
            motors.get(0).setPower(1);
        } else if (goingout) {
            motors.get(0).setPower(-1);
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