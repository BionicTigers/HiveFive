//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//public class Drivetrain extends Mechanism {
//    //Declares values
//    public org.firstinspires.ftc.teamcode.Robot robot;
//    public double[] motorPowers;
//    public int[] motorIndices;
//
//    //Constructor method
//    public Drivetrain(org.firstinspires.ftc.teamcode.Robot bot, int[] motorNumbers) {
//        DcMotorEx motorPlaceholder;
//        robot = bot;
//        motorIndices = motorNumbers;
//
//        for (int motNum : motorNumbers) {
//            motorPlaceholder = robot.motors.get(motNum);
//            motorPlaceholder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            motors.add(motorPlaceholder);
//        }
//        motorPowers = new double[]{0, 0, 0, 0};
//    }
//        //sets the motorNumbers array based on input from joysticks
//        public void determineMotorPowers (Gamepad driverPad){
//            double P = Math.hypot(-driverPad.left_stick_x, -driverPad.left_stick_y);
//            double robotAngle = Math.atan2(-driverPad.left_stick_y, -driverPad.left_stick_x);
//            double rightX = driverPad.right_stick_x;
//
//            double sinRAngle = Math.sin(robotAngle);
//            double cosRAngle = Math.cos(robotAngle);
//
//            final double v1 = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontRight
//            final double v2 = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontLeft
//            final double v3 = (P * sinRAngle) + (P * cosRAngle) + rightX;  //backRight
//            final double v4 = (P * sinRAngle) - (P * cosRAngle) - rightX;  //backLeft
//
//            motorPowers[0] = v1;
//            motorPowers[1] = v2;
//            motorPowers[2] = v3;
//            motorPowers[3] = v4;
//        }
//
//        //This code was in the wrong place, it was in the constructor method when it should be after.
//
////        //Uses values from motor powers array to move the robot
////        public void robotMovement () {
////
////        }
////        public void update (Gamepad gp1, Gamepad gp2){
////            determineMotorPowers(gp1);
////        }
////
////        public void write () {
////            //Sets the motor powers based on the determineMotorPowers() method that was run in the update() method
////            int i = 0;
////            for (DcMotorEx motor : motors.subList(motorIndices[0], motorIndices[3] + 1)) {
////                motor.setPower(motorPowers[i]);
////                i++;
////            }
////        }
//    }