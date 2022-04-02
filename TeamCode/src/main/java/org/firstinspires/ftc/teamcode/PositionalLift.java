package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.AbstractCollection;

public class PositionalLift extends Mechanism {
    public String position = "Mid";
    public Telemetry telemetry;
    public DcMotorEx motor;
    private boolean up;
    private boolean mid;
    private boolean liftOverride;
    private boolean down;

    public PositionalLift(DcMotorEx m, Telemetry T){
        super();
        telemetry = T;
        telemetry.addData("<", "Fully initialized");
        DcMotorEx motor = m;
        motors.add(motor);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(0).setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void update(Gamepad gp1, Gamepad gp2)
    {
        if(gp2.right_trigger >= .2)
        {
            up = true;
            mid = false;
            down = false;
            liftOverride = false;
        }
        else if(gp2.start)
        {
            mid = true;
            up = false;
            down = false;
            liftOverride = false;
        }
        else if (gp2.left_trigger > .2) {
            mid = false;
            up = false;
            down = true;
            liftOverride = false;
        }

        if(gp2.start && gp2.back){
            liftOverride = true;
        }

        telemetry.addData("up:", up);
        telemetry.addData("mid:", mid);
        telemetry.addData("down:", down);
        telemetry.addData("override:", liftOverride);
        telemetry.addData("lift position: ", motors.get(0).getCurrentPosition() );
        telemetry.addData("lift power: ", motors.get(0).getPower() );

        telemetry.update();

    }
    public void write(){
        if(!liftOverride)
        {
            motors.get(0).setPower(100);
        }
        else motors.get(0).setPower(0);
        if (up) motors.get(0).setTargetPosition(1200);
        else if (mid) motors.get(0).setTargetPosition(600);
        else if (down) motors.get(0).setTargetPosition(0);
    }
}
