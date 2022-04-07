package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class Turret extends Mechanism{

    public boolean forward;
    public boolean left;
    public boolean backward;
    public boolean right;

    public boolean altMode = false;
    public int spinTrim = 0;
    public int verticalTrim = 0;
    public double horizontalTrim = 0;
    public int trim = 0;
    public boolean extend = false;
    public boolean middle = false; //middle is for extension, do not confuse with mid
    public boolean retract = false;

    public DcMotorEx elevator;
    public DcMotorEx extender;
    public DcMotorEx rotate;

    public Telemetry telemetry;
    public DcMotorEx motor;
    private boolean up;
    private boolean mid; //mid is for height, do not confuse with middle
    private boolean liftOverride;
    private boolean down;

    Deadline wait = new Deadline (500, TimeUnit.MILLISECONDS);

    public Turret(DcMotorEx elevator, DcMotorEx extender, DcMotorEx rotate, Telemetry T){
        super();
        telemetry = T;
        this.elevator = elevator;
        this.extender = extender;
        this.rotate = rotate;
        motors.add(elevator);
        motors.add(extender);
        motors.add(rotate);
        motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(0).setTargetPosition(0);
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.get(1).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(1).setTargetPosition(0);
        motors.get(1).setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveElevator(int distance) {
        elevator.setTargetPosition(distance);
    }
    public void moveExtender(int distance) {
        extender.setTargetPosition(distance);
    }
    public void rotateTurret(int angle) {
        rotate.setTargetPosition(angle);
    }

    public void update(Gamepad gp1, Gamepad gp2){
        if(gp2.dpad_up){
            forward = true;
        } else if(gp2.dpad_left){
            left = true;
        } else if(gp2.dpad_down){
            backward = true;
        } else if(gp2.dpad_right){
            right = true;
        } else {
            forward = false;
            left = false;
            backward = false;
            right = false;
        }
        if (gp1.right_bumper && gp1.dpad_up) {
            altMode = true;
        }
        if (gp1.right_bumper && gp1.dpad_down) {
            altMode = false;
        }

        if (altMode && gp1.right_stick_x >= 0.3) {
            spinTrim = spinTrim + 5;
        } else if (altMode && gp1.right_stick_x <= -0.3) {
            spinTrim = spinTrim - 5;
        }
        if (gp2.right_stick_y <= -0.5 && gp2.right_bumper) {
            extend = true;
            retract = false;
            middle = false;
        } else if (gp2.right_stick_y >= 0.5) {
            extend = false;
            retract = true;
            middle = false;
        } else if (gp2.right_stick_y <= -0.5 && !gp2.right_bumper) {
            middle = true;
            extend = false;
            retract = false;
        }
        if(gp2.b)
        {
            up = true;
            mid = false;
            down = false;
            liftOverride = false;
        }
        else if(gp2.y)
        {
            mid = true;
            up = false;
            down = false;
            liftOverride = false;
        }
        else if (gp2.x) {
            mid = false;
            up = false;
            down = true;
            liftOverride = false;
        }

        if(gp2.start && gp2.back){
            liftOverride = true;
        }
        if(gp2.right_stick_button && gp2.left_stick_button){
            unfold();
        }

        if (gp2.left_stick_y <= -0.5) {
            horizontalTrim = horizontalTrim + 0.002;
        }
        if (gp2.left_stick_y >= 0.5) {
            horizontalTrim = horizontalTrim - 0.002;
        }
        if (gp2.right_trigger >= 0.5) {
            verticalTrim = verticalTrim + 5;
        }
        if (gp2.left_trigger >= 0.5) {
            verticalTrim = verticalTrim - 5;
        }

        telemetry.addData("up:", up);
        telemetry.addData("mid:", mid);
        telemetry.addData("down:", down);
        telemetry.addData("override:", liftOverride);
        telemetry.addData("lift position: ", motors.get(0).getCurrentPosition() );
        telemetry.addData("lift power: ", motors.get(0).getPower() );
    }

    public void write(){
        if(forward){
            motors.get(0).setTargetPosition(2590 + spinTrim);
            motors.get(0).setPower(60);
        } else if(left && !(retract && down)){
            motors.get(0).setTargetPosition(588 + spinTrim);
            motors.get(0).setPower(60);
        } else if(backward && !down){
            motors.get(0).setTargetPosition(1600 + spinTrim);
            motors.get(0).setPower(60);
        } else if(right && !(retract && down)){
            motors.get(0).setTargetPosition(4557 + spinTrim);
            motors.get(0).setPower(60);
        }
        if (extend) {
            servos.get(0).setPosition(0.9 + horizontalTrim); //0.08
            servos.get(1).setPosition(0.1 - horizontalTrim); //0.58
       } else if (retract) {
            servos.get(0).setPosition(1); //0.32
            servos.get(1).setPosition(0); //0.35
        } else if (middle){
            servos.get(0).setPosition(0.8 + horizontalTrim);
            servos.get(1).setPosition(0.2 - horizontalTrim);
        }
        if(!liftOverride)
        {
            motors.get(1).setPower(100);
        }
        else motors.get(1).setPower(0);

        if (up) {
            motors.get(1).setTargetPosition(1800 + verticalTrim);
        } else if (mid) {
            motors.get(1).setTargetPosition(900 + verticalTrim);
        } else if (down) {
            motors.get(1).setTargetPosition(50 + verticalTrim);
        }
    }
    public void unfold(){
        wait.reset();
        while(!wait.hasExpired()) {
            servos.get(0).setPosition(0.8);
            servos.get(1).setPosition(0.2);
        }
        motors.get(0).setTargetPosition(2590 + spinTrim);
        motors.get(0).setPower(60);
        servos.get(0).setPosition(1);
        servos.get(1).setPosition(0);
    }
}
