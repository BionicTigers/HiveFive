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

    /**
     * Moves the arm to the bottom
     * @param motor motor that controls transfer arm
     */
    public void moveToBottom(DcMotorEx motor){
        motor.setTargetPosition(-250);
    }
    /**
     * Moves the arm to the middle
     * @param motor motor that controls transfer arm
     */
    public void moveToMiddle(DcMotorEx motor){
        motor.setTargetPosition(-500);
    }
    /**
     * Moves the arm to the top
     * @param motor motor that controls transfer arm
     */
    public void moveToTop(DcMotorEx motor){
        motor.setTargetPosition(-773);
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
            motors.get(0).setTargetPosition(-840);
        } else if(down){
            motors.get(0).setPower(30);
            motors.get(0).setTargetPosition(0);
        }
    }

}
