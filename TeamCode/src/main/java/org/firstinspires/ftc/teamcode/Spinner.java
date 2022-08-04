package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class Spinner extends Mechanism {

        //Controller buttons
        public boolean aIsPressed;
        public boolean spinning;
        public int x = 0;
        public int startSpot = 0;
        Deadline timer = new Deadline(5, TimeUnit.SECONDS);

        //Used to declare new instances of Spinner
        public Spinner(DcMotorEx spinner) {
            super();
            motors.add(spinner);
            motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors.get(0).setTargetPosition(0);
            motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void update(Gamepad gp1, Gamepad gp2) {
            if (gp2.dpad_right) {
                autoSpin();
                aIsPressed = true;
            }

        }

        public void write() {
            if (spinning && motors.get(0).getCurrentPosition() <= startSpot - (1300*4/3)) {
                motors.get(0).setVelocity(2500);
            }

        }

        public void autoSpin() {
            timer.reset();
            while(!timer.hasExpired()) {
                motors.get(0).setPower(1);
                spinning = true;
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