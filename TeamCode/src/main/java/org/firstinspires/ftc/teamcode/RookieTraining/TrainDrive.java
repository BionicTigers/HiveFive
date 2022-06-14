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

    private int moveDirection;
    private float movePower;
    //Constructor
    public TrainDrive(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx  br){
        frontRight = fr;
        frontLeft = fl;
        backRight = br;
        backLeft = bl;
    }
    //Methods
    private void moveForwardRight(float power){
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }
    private void moveBackwardRight(float power){
        frontRight.setPower(power);
        backRight.setPower(power);
    }
    private void moveForwardLeft(float power){
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
    }
    private void moveBackwardLeft(float power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }
    public void moveForward(float power) {
        moveForwardLeft(power);
        moveForwardRight(power);
    }
    public void moveBackward(float power) {
        moveBackwardLeft(power);
        moveBackwardRight(power);
    }
    public void turnRight(float power) {
        moveForwardRight(power);
        moveBackwardLeft(power);
    }
    public void turnLeft(float power) {
        moveForwardLeft(power);
        moveBackwardRight(power);
    }
    public void stop() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp1.left_stick_y > 0.3) {
            moveDirection = 1;
            movePower = Math.abs(gp1.left_stick_y);
        } else if (gp1.left_stick_y < -0.3) {
            moveDirection = 3;
            movePower = Math.abs(gp1.left_stick_y);
        } else if (gp1.left_stick_x > 0.3) {
            moveDirection = 4;
            movePower = Math.abs(gp1.left_stick_x);
        } else if (gp1.left_stick_x < -0.3) {
            moveDirection = 2;
            movePower = Math.abs(gp1.left_stick_x);
        }
    }

    public void write() {
        switch (moveDirection) {
            case 1: //Forward
                moveForward(movePower);
            case 2: //Left
                turnLeft(movePower);
            case 3: //Backwards
                moveBackward(movePower);
            case 4: //Right
                turnRight(movePower);
            default:
                stop();
        }
    }
}
