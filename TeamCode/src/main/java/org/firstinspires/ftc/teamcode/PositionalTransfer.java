package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;

public class PositionalTransfer extends Mechanism{
    public String position = "Mid";
    public Telemetry telemetry;
    public DcMotorEx motor;

    public PositionalTransfer(DcMotorEx m, Telemetry T){
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
        if(gp2.right_trigger >= 0.5){
            position = "Up";
        } else if(gp2.left_trigger >= 0.3){
            position =  "Mid";
        }
        else if(gp1.right_trigger >= .5){
            position = "Down";
        }


        telemetry.addData("position", motors.get(0).getCurrentPosition());
//        telemetry.update();
    }

    public void write(){
        if(position == "Up"){
            motors.get(0).setPower(80);
            motors.get(0).setTargetPosition(-2280);
        }
        else if(position == "Down") {
            motors.get(0).setPower(50);
            motors.get(0).setTargetPosition(0);
        }
        else if (position == "Mid"){
            motors.get(0).setPower(50);
            motors.get(0).setTargetPosition(-600);
        }
        if (position == "Down")
        {
            position = "Mid";
        }
    }
}
