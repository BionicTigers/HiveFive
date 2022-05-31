package org.firstinspires.ftc.teamcode.RookieTraining;
/*
    Quest:
        Create a TrainDrive object
        Create 4 motors
        Be able to move forward and backwards
        Be able to turn left and right
 */

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Mechanism;

public class TrainDrive extends Mechanism {
    //Fields
    public DcMotorEx frontRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backRight;
    public DcMotorEx backLeft;
    //Constructor
    public TrainDrive(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx  br){
        frontRight = fr;
        frontLeft = fl;
        backRight = br;
        backLeft = bl;
    }
    //Methods
    public void moveForwardRight(){
        frontRight.setPower(-1);
        backRight.setPower(-1);
    }
    public void moveBackwardRight(){
        frontRight.setPower(1);
        backRight.setPower(1);
    }
    public void moveForwardLeft(){
        frontLeft.setPower(-1);
        backLeft.setPower(-1);
    }
    public void moveBackwardLeft(){
        frontLeft.setPower(1);
        backLeft.setPower(1);
    }
    @Override
    public void update(Gamepad gp1, Gamepad gp2) {

    }

    @Override
    public void write() {

    }
}
