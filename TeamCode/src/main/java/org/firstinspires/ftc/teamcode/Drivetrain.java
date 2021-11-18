package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.hardware.bosch.BNO055IMU;

/*
This class declares the drivetrain mechanism and sends data from the controller to the robot and
 uses that data to set the motor powers
 */

public class Drivetrain extends Mechanism {
    //Declares values
    public org.firstinspires.ftc.teamcode.Robot robot;
    public double[] motorPowers;
    public int[] motorIndices;
    public Telemetry telemetree;
    private Odometry odo;
    private Location forward = new Location (0, 0, 1000, 0);
    private Location backward = new Location (0, 0, -1000, 0);
    private Location left = new Location (-500, 0, 0, 0);
    private Location right = new Location (500, 0, 0, 0);
    private Location clockwise = new Location (0, 0, 0, 90);
    private Location counterclockwise = new Location (0, 0, 0, 270);
    private Location center = new Location (0, 0, 0, 0);

    //Constructor method
    public Drivetrain(@NonNull org.firstinspires.ftc.teamcode.Robot bot, @NonNull int[] motorNumbers, Telemetry T, Servo servo) {
        DcMotorEx motorPlaceholder;
        robot = bot;
        motorIndices = motorNumbers;
        telemetree = T;
        odo = bot.odometry;
        getServos().add(servo);

        for (int motNum : motorNumbers) {
            motorPlaceholder = robot.motors.get(motNum);
            motorPlaceholder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.add(motorPlaceholder);
        }
        motorPowers = new double[]{0, 0, 0, 0};
    }
        //sets the motorNumbers array based on input from joysticks
        public void determineMotorPowers (Gamepad driverPad){
            double P = Math.hypot(-driverPad.left_stick_x, -driverPad.left_stick_y);
            double robotAngle = Math.atan2(-driverPad.left_stick_y, -driverPad.left_stick_x);
            double rightX = driverPad.right_stick_x;

            double sinRAngle = Math.sin(robotAngle);
            double cosRAngle = Math.cos(robotAngle);

            final double v1 = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontRight
            final double v2 = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontLeft
            final double v3 = (P * sinRAngle) + (P * cosRAngle) + rightX;  //backRight
            final double v4 = (P * sinRAngle) - (P * cosRAngle) - rightX;  //backLeft

            motorPowers[0] = v1;
            motorPowers[1] = v2;
            motorPowers[2] = v3;
            motorPowers[3] = v4;
        }
        public void determineServoMovement(Gamepad driverPad){
            if (driverPad.a){
                servos.get(0).setPosition(0.6);

            } else if(driverPad.b){
                servos.get(0).setPosition(0.67);
            }
        }
        //Uses values from motor powers array to move the robot
        public void robotMovement () {

        }
        public void update (Gamepad gp1, Gamepad gp2){
            /*
            All telemetree commands implement telemetry to allow the driver to view motor powers
            while code is active
             */
            telemetree.addLine("Motor Powers");
            telemetree.addData("Front Right Power", motorPowers[0]);
            telemetree.addData("Front Left Power", motorPowers[1]);
            telemetree.addData("Back Right Power", motorPowers[2]);
            telemetree.addData("Back Left Power", motorPowers[3]);
            telemetree.update();
            determineMotorPowers(gp1); //Updates values in motorPowers array
            determineServoMovement(gp1);
        }

        public void write () {
            //Sets the motor powers based on the determineMotorPowers() method that was run in the update() method
            int i = 0;
            for (DcMotorEx motor : motors.subList(motorIndices[0], motorIndices[3] + 1)) {
                motor.setPower(motorPowers[i]);
                i++;
           }
       }
    }