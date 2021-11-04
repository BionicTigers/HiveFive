package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Mechanism;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class RookieDrivetrain extends Mechanism {

    //Declares values
    public org.firstinspires.ftc.teamcode.Robot rookieBot;
    public double[] motorPowers;
    public int[] motorIndices;


    //idk what it does, but it does
    /*LOCATIONS NOT NECESSARY, THAT IS FOR AUTOS*/
    //but it doesn't tho
    /*private Location forth = new Location (0, 0, 1000, 0);
    private Location backth = new Location (0, 0, -1000, 0);
    private Location lefth = new Location (-500, 0, 0, 0);
    private Location righth = new Location (500, 0, 0, 0);
    private Location clockth = new Location (0, 0, 0, 90);
    private Location counterth = new Location (0, 0, 0, 270);
    private Location centerth = new Location (0, 0, 0, 0);*/

    //Constructor method
    /*Use more effective and desrciptive names. I like the funny, but make them descriptive as to what the variable actually is.*/
    public RookieDrivetrain(org.firstinspires.ftc.teamcode.Robot bot, int[] motorNumbers) {
        DcMotorEx motorPlaceholder;
        rookieBot = bot;
        motorIndices = motorNumbers;

        for (int motNum : motorNumbers) {
            motorPlaceholder = rookieBot.motors.get(motNum);
            motorPlaceholder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.add(motorPlaceholder);
        }
        motorPowers = new double[]{0, 0, 0, 0};

        /*FTC dashboard is nothing you need to worry about right now. U can google it if u want to know what's up*/
        //rookieDashboard = FtcDashboard.getInstance();


    }

    //sets the motor powers array depending on input from joysticks
    public void determineMotorPowers(Gamepad driverPad){
        double P = Math.hypot(-driverPad.left_stick_x, -driverPad.left_stick_y);
        double robotAngle = Math.atan2(-driverPad.left_stick_y, -driverPad.left_stick_x);
        double rightX = driverPad.right_stick_x;

        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v1 = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontRight
        final double v2 = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontLeft
        final double v3 = (P * sinRAngle) + (P * cosRAngle) + rightX;  //backRight
        final double v4 = (P * sinRAngle) - (P * cosRAngle) - rightX;  //backLeft

        motorPowers[0] = v1; motorPowers[1] = v2; motorPowers[2] = v3; motorPowers[3] = v4;


    }
    //Uses values from motor powers array to move the robot
    public void robotMovement(){

    }
    public void update(Gamepad gp1, Gamepad gp2) {
        determineMotorPowers(gp1);
    }

    public void write(){
        //Sets the motor powers based on the determineMotorPowers() method that was run in the update() method
        int i = 0;
        for(DcMotorEx motor:motors.subList(motorIndices[0], motorIndices[3] + 1)){
            motor.setPower(motorPowers[i]);
            i++;
        }
    }
}