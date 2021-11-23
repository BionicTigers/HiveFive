package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Speener extends Mechanism{
    
    public boolean speen;
    public Speener(DcMotorEx spinning) {
        super();
        motors.add(spinning);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        speen = gp2.y;
    }

    public void write() {
        if (speen) {
            motors.get(0).setPower(1);
        } else {
            motors.get(0).setPower(0);
        }
    }
}

/*
    Pseudocode:
    (Potentially) Move automatically to the corner
    DIRECTIVE: Spin
    Ducky Profit
    Stop so duck resupply can occur
    Repeat last few steps
*/