package org.firstinspires.ftc.teamcode.RookieTraining;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanism;

public class PracticeClass extends Mechanism {

    public PracticeClass(DcMotorEx motor, Servo servo){
        motors.add(motor);
        servos.add(servo);
    }

    public boolean rookie;
    public boolean buttons;

    public void update(Gamepad gp1, Gamepad gp2) {
        if(gp1.a){
            rookie = true;
        }



    }

    public void write() {
        if(rookie){
            motors.get(0).setPower(50);
        } else{
            motors.get(0).setPower(0);
        }
    }
}
