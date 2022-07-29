package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;


public  class Robot {
    public LinearOpMode linoop;
    public OpMode oop;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public Telemetry telemetry;
    public ArrayList<DcMotorEx> motors;
    public ArrayList<Servo> servos;
    public ArrayList<CRServo> crServos;
    public ArrayList<DigitalChannel> sensors;
    public ElapsedTime time;
    public HardwareMap hardwareMap;
    public Odometry odometry;


    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"};


     //Robot constructor class; creates robot object
    public Robot(OpMode opMode) {
        oop = opMode;
        hardwareMap = oop.hardwareMap;

        motors = new ArrayList<>();
        motors.add(hardwareMap.get(DcMotorEx.class,"frontLeft"));
        motors.add(hardwareMap.get(DcMotorEx.class,"frontRight"));
        motors.add(hardwareMap.get(DcMotorEx.class,"backLeft"));
        motors.add(hardwareMap.get(DcMotorEx.class,"backRight"));
        motors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(1).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(3).setDirection(DcMotorSimple.Direction.REVERSE);
        time = new ElapsedTime();
//
        gamepad1 = oop.gamepad1;
        gamepad2 = oop.gamepad2;
//
        telemetry = oop.telemetry;
//        dt = new Drivetrain(this, new int[]{0, 1, 2, 3});
//


//        odometry = new Odometry(hardwareMap);
    }


     //Robot constructor class; creates robot object
    public Robot(LinearOpMode opMode) {

        linoop = opMode;
        hardwareMap = linoop.hardwareMap;

        motors = new ArrayList<>();
        motors.add(hardwareMap.get(DcMotorEx.class,"frontRight"));
        motors.add(hardwareMap.get(DcMotorEx.class,"frontLeft"));
        motors.add(hardwareMap.get(DcMotorEx.class,"backLeft"));
        motors.add(hardwareMap.get(DcMotorEx.class,"backRight"));
        motors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(3).setDirection(DcMotorSimple.Direction.REVERSE);
        time = new ElapsedTime();

        gamepad1 = linoop.gamepad1;
        gamepad2 = linoop.gamepad2;

        telemetry = linoop.telemetry;
//        dt = new Drivetrain(this, new int[]{0, 1, 2, 3});



        odometry = new Odometry(hardwareMap);
    }

    public Robot(){}

     //Returns the time
    public long getTimeMS(){
        return time.now(TimeUnit.MILLISECONDS);
    }

    public void initMotors (String[]motorNames){
        for (String motor : motorNames) {
            motors.add(hardwareMap.get(DcMotorEx.class, motor));
        }
    }
//    public DigitalChannel[] getSensor1() { return DigitalChannel[0]; }
    //abstract update method
    //public abstract void update (Gamepad gp1, Gamepad gp2);
}