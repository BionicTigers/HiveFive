package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PositionalTransferFine extends Mechanism{
    public String position = "Mid";
    public Telemetry telemetry;
    public DcMotorEx motor;
    public int x = 0;

    public PositionalTransferFine(DcMotorEx m, Telemetry T){
        super();
        DcMotorEx motor = m;
        motors.add(motor);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(0).setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry = T;
    }

     //Moves the arm to the bottom
    public void moveToBottom(){
        motor.setTargetPosition(-250);
    }

     //Moves the arm to the middle
    public void moveToMiddle(){
        motor.setTargetPosition(-500);
    }
    //Moves the arm to the top
    public void moveToTop(){
        motor.setTargetPosition(-773);
    }

    public void update(Gamepad gp1, Gamepad gp2){
        if(gp1.right_bumper){
            x = x - 1;
        } else if(gp1.left_bumper){
            x = x + 1;
        }
        if (x > 0) {
            x = 0;
        }

        telemetry.addData("position", motors.get(0).getCurrentPosition());
//        telemetry.update();
    }

    public void write(){
        motors.get(0).setPower(50);
        motors.get(0).setTargetPosition(x);
    }
}
