//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//@TeleOp(name = "wheels")
//public class WheelTroubleshooting extends LinearOpMode{
//    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"}; //establishes motor names
//    public Drivetrain drivetrain; //declares drivetrain
//    public Robot robot; //declares robot
//    public int[] motorNumbers = {0, 1, 2, 3};
//    public void runOpMode(){
//        robot = new Robot(this);
//        waitForStart();
//        while(opModeIsActive()){
//        drivetrain = new Drivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
//        if(gamepad2.dpad_up){
//            drivetrain.motors.get(0).setPower(1); //frontRight
//        }
//        if(gamepad2.dpad_right){
//            drivetrain.motors.get(1).setPower(1); //frontLeft
//        }
//        if(gamepad2.dpad_down){
//            drivetrain.motors.get(2).setPower(1); //backLeft
//        }
//        if(gamepad2.dpad_left){
//            drivetrain.motors.get(3).setPower(1);
//        }
//    }}
//}
