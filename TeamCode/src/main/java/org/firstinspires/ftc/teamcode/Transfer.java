package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Transfer extends Mechanism{

    boolean up;
    boolean down;

    public Transfer(DcMotorEx transferMotor){
        super();
        motors.add(transferMotor);
    }

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
