package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Turret extends Mechanism{

    public boolean forward;
    public boolean left;
    public boolean backward;
    public boolean right;

    public boolean altMode = false;
    public int trim = 0;

    public Turret(DcMotorEx turret){
        super();
        motors.add(turret);
        motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(0).setTargetPosition(0);
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void update(Gamepad gp1, Gamepad gp2){
        if(gp2.dpad_up){
            forward = true;
        } else if(gp2.dpad_left){
            left = true;
        } else if(gp2.dpad_down){
            backward = true;
        } else if(gp2.dpad_right){
            right = true;
        } else{
            forward = false;
            left = false;
            backward = false;
            right = false;
        }
        if (gp1.right_bumper && gp1.dpad_up) {
            altMode = true;
        }
        if (gp1.right_bumper && gp1.dpad_down) {
            altMode = false;
        }

        if (altMode && gp1.right_stick_x >= 0.3) {
            trim = trim + 5;
        } else if (altMode && gp1.right_stick_x <= -0.3) {
            trim = trim - 5;
        }
    }

    public void write(){
        if(forward){
            motors.get(0).setTargetPosition(0 + trim);
            motors.get(0).setPower(60);
        } else if(left){
            motors.get(0).setTargetPosition(2000 + trim);
            motors.get(0).setPower(60);
        } else if(backward){
            motors.get(0).setTargetPosition(1600 + trim);
            motors.get(0).setPower(60);
        } else if(right){
            motors.get(0).setTargetPosition(-2000 + trim);
            motors.get(0).setPower(60);
        }
    }
}
