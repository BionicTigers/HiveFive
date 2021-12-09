package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PositionalTransfer extends Mechanism{
    boolean up;
    boolean down;
    public Telemetry telemetry;

    public PositionalTransfer(DcMotorEx motor, Telemetry T){
        super();
        motors.add(motor);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(0).setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry = T;
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
        telemetry.addData("position", motors.get(0).getCurrentPosition());
        telemetry.update();
    }

    public void write(){
        if(up){
            motors.get(0).setPower(30);
            motors.get(0).setTargetPosition(-773);
        } else if(down){
            motors.get(0).setPower(30);
            motors.get(0).setTargetPosition(0);
        }
    }

}
