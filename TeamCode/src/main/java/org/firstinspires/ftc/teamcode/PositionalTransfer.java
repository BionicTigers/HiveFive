package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;

public class PositionalTransfer extends Mechanism{
    boolean up;
    boolean down;
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

    /*
     * Moves the arm to the bottom
     */
    public void moveToBottom(){
        motor.setTargetPosition(-250);
    }
    /*
     * Moves the arm to the middle
     */
    public void moveToMiddle(){
        motor.setTargetPosition(-500);
    }
    /*
     * Moves the arm to the top
     */
    public void moveToTop(){
        motor.setTargetPosition(-773);
    }

    /*
     * Moves arm to the best height
     */
    public void moveToBest() {
        switch (Variables.lev) {
            case BOTTOM:
                moveToBottom();
                break;
            case MIDDLE:
                moveToMiddle();
                break;
            case TOP:
                moveToTop();
                break;
        }
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
//        telemetry.update();
    }

    public void write(){
        if(up){
            motors.get(0).setPower(30);
            motors.get(0).setTargetPosition(-840);
        } else if(down){
            motors.get(0).setPower(30);
            motors.get(0).setTargetPosition(0);
        }
    }

}
