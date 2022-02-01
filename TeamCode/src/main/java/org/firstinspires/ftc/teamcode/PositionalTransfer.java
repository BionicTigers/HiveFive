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
    public int trim = 0;

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
            position = "Mid";
        } else if((gp1.right_trigger >= .5  && position != "Up" )|| gp2.dpad_down){
            position = "Down";
        }

        if (gp2.right_bumper) {
            trim = trim - 1;
        } else if (gp2.left_bumper) {
            trim = trim + 1;
        }

        if (gp2.right_stick_button) {
            trim = trim + 3;
        }

        telemetry.addData("position", motors.get(0).getCurrentPosition());
//        telemetry.update();
    }

    public void write(){
        if(position == "Up"){
            motors.get(0).setPower(100);
            motors.get(0).setTargetPosition(-2240 + trim);
        }
        else if(position == "Down") {
            motors.get(0).setPower(100);
            motors.get(0).setTargetPosition(0);
        }
        else if (position == "Mid"){
            motors.get(0).setPower(100);
            motors.get(0).setTargetPosition(-600 + trim);
        }
        if (position == "Down")
        {
            position = "Mid";
        }
    }
}
