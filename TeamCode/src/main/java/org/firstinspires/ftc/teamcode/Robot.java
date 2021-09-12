package org.firstinspires.ftc.teamcode.Mechanisms;

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
    public LinearOpMode Linoop;
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

    //Robot constructor class; creates robot object
    public Robot(OpMode opMode) {
        oop = opMode;
        hardwareMap = oop.hardwareMap;

        motors = new ArrayList<>();
        motors.add((DcMotorEx)hardwareMap.get(DcMotor.class,"frontLeft"));
        motors.add((DcMotorEx)hardwareMap.get(DcMotor.class,"frontRight"));
        motors.add((DcMotorEx)hardwareMap.get(DcMotor.class,"backLeft"));
        motors.add((DcMotorEx)hardwareMap.get(DcMotor.class,"backRight"));
        motors.get(1).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(3).setDirection(DcMotorSimple.Direction.REVERSE);
        time = new ElapsedTime();
//
        gamepad1 = oop.gamepad1;
        gamepad2 = oop.gamepad2;
//
        hardwareMap = oop.hardwareMap;
        telemetry = oop.telemetry;
//        dt = new Drivetrain(this, new int[]{0, 1, 2, 3});
//
    }
    public  Robot(LinearOpMode opMode) {
        oop = opMode;
        Linoop = opMode;

        hardwareMap = oop.hardwareMap;
//
        motors = new ArrayList<>();
        motors.add((DcMotorEx)hardwareMap.get(DcMotor.class,"frontLeft"));
        motors.add((DcMotorEx)hardwareMap.get(DcMotor.class,"frontRight"));
        motors.add((DcMotorEx)hardwareMap.get(DcMotor.class,"backLeft"));
        motors.add((DcMotorEx)hardwareMap.get(DcMotor.class,"backRight"));
        motors.get(1).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(3).setDirection(DcMotorSimple.Direction.REVERSE);
        time = new ElapsedTime();
//
        gamepad1 = oop.gamepad1;
        gamepad2 = oop.gamepad2;
//
        hardwareMap = oop.hardwareMap;
        telemetry = oop.telemetry;
//        dt = new Drivetrain(this, new int[]{0, 1, 2, 3});
//
    }

    public Robot() {}

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