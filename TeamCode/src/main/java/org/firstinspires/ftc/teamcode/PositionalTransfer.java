package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class PositionalTransfer extends Mechanism{
    boolean up;
    boolean down;

    public PositionalTransfer(DcMotorEx motor){
        super();
        motors.add(motor);
    }

    public void update(Gamepad gp1, Gamepad gp2){
        if(gp2.right_trigger >= 0.5){
            up = true;
        } else if(gp2.left_trigger >= 0.5){
            down = true;
        } else{
            up = false;
            down = false;
        }
    }

    public void write(){
        if(up){
            motors.get(0).setTargetPosition(500);
        } else if(down){
            motors.get(0).setTargetPosition(0);
        }
    }

}
