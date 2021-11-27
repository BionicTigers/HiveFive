package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Transfer extends Mechanism{

    //variables used to determine what the motor is doing
    boolean up;
    boolean down;

    //establishes the motor
    public Transfer(DcMotorEx transferMotor){
        super();
        motors.add(transferMotor);
    }

    //determines the true/false value of the up and down variables
    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if(gp1.dpad_up){
            up = true;
        } else if(gp1.dpad_down){
            down = true;
        } else{
            up = false;
            down = false;
        }
    }

    //using the values of up and down, determines what to do next
    @Override
    public void write() {
        if(up){
            motors.get(0).setPower(60);
        } else if(down){
            motors.get(0).setPower(-60);
        } else{
            motors.get(0).setPower(0);
        }
    }
}
