package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Transfer extends Mechanism{

    public Transfer(DcMotorEx transferMotor){
        super();
        motors.add(transferMotor);
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if(gp1.a){
            motors.get(0).setPower(60);
        } else if(gp1.b){
            motors.get(0).setPower(-60);
        } else{
            motors.get(0).setPower(0);
        }
    }

    @Override
    public void write() {

    }
}
