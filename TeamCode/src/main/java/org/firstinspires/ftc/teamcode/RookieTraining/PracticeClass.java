package org.firstinspires.ftc.teamcode.RookieTraining;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanism;

public class PracticeClass extends Mechanism {

    public PracticeClass(DcMotorEx motor, Servo servo, CRServo continuous){
        motors.add(motor);
        servos.add(servo);
        crServos.add(continuous);
    }

    public boolean rookie;
    public boolean buttons;

    public void update(Gamepad gp1, Gamepad gp2) {
        if(gp1.a && !gp1.b){
            rookie = true;
        } else {
            rookie = false;
        }

        if (gp1.a && gp1.b) {
            buttons = true;
        } else {
            buttons = false;
        }

        if (gp1.a && gp1.b) {
            buttons = true;
            rookie = false;
        } else if (gp1.a) {
            rookie = true;
            buttons = false;
        } else {
            buttons = false;
            rookie = false;
        }

        rookie = gp1.a && !gp1.b;
        buttons = gp1.a && gp1.b;

        int num = 3;
        boolean isTrue = false;
        boolean isFalse = true;

        //Single line
        /*
        multi line
         */
        /**
         * Forbidden comment?
         */
        switch(num){
            case 1:
                isTrue = true;
                break;
            case 2:
                isFalse = false;
                break;
            default:
                isTrue = false;
                isFalse = true;
                break;
        }

        int big = 0;

        //2.5 types of loops
        //while
        while(gp1.y){
            big = big + 1;
            big ++;
            big += 1;
            big = big - 1;
            big -= 1;
            big --;
            big *= 1;
            big /= 1;
        }
        int[] arr = new int[5];
        //for
        for(int i = 0; i > 0;) {
            isTrue = false;
            arr[i] = 5;
        }
        //for each
        for(int a: arr) {

        }
    }

    public void write() {
        if(rookie){
            motors.get(0).setPower(0.5);
        } else{
            motors.get(0).setPower(0);
        }

        if (buttons) {
            servos.get(0).setPosition(0.5);
            crServos.get(0).setPower(1);
        } else {
            crServos.get(0).setPower(0);
            servos.get(0).setPosition(0.1);
        }
    }
}
