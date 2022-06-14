package org.firstinspires.ftc.teamcode.RookieTraining;
/*
    Your mission, should you choose to accept it:
        Create a TrainDrive object
        Create 4 motors
        Be able to move forward and backwards
        Be able to turn left and right
 */

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Mechanism;

/**
 * Drives Tank
 */
public class DriveTank extends Mechanism {
    //Fields
    public DcMotorEx frontRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backRight;
    public DcMotorEx backLeft;

    private int moveDirectionLeft;
    private int moveDirectionRight;
    private float movePowerLeft;
    private float movePowerRight;

    //Constructor
    /**
     * Creates a new DriveTank object that can be used to drive the robot
     * @param fl front left motor
     * @param fr front right motor
     * @param bl back left motor
     * @param br back right motor
     */
    public DriveTank(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx  br){
        frontRight = fr;
        frontLeft = fl;
        backRight = br;
        backLeft = bl;
    }
    //Methods

    /**
     * Power?
     * @param power power power power
     */
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
    public void stopLeft() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
    }
    public void stopRight() {
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp1.left_stick_y > 0.3) {
            moveDirectionLeft = 1;
            movePowerLeft = Math.abs(gp1.left_stick_y);
        } else if (gp1.left_stick_y < -0.3) {
            moveDirectionLeft = 2;
            movePowerLeft = Math.abs(gp1.left_stick_y);
        }
        if (gp1.right_stick_y > 0.3) {
            moveDirectionRight = 1;
            movePowerRight = Math.abs(gp1.right_stick_y);
        } else if (gp1.right_stick_y < -0.3) {
            moveDirectionRight = 2;
            movePowerRight = Math.abs(gp1.right_stick_y);
        }
    }

    public void write() {
        switch (moveDirectionLeft) {
            case 1: //Forward
                moveForwardLeft(movePowerLeft);
            case 2: //Backward
                moveBackwardLeft(movePowerLeft);
        }
        switch (moveDirectionRight) {
            case 1: //Forward
                moveForwardLeft(movePowerRight);
            case 2: //Backward
                moveBackwardLeft(movePowerRight);
        }
    }
}
