package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class ConciseIntake extends Mechanism {
    Deadline stop = new Deadline(500, TimeUnit.MILLISECONDS);

    //Both of these variables control the direction of the intake
    public boolean intaking;
    public boolean outputting;
    public boolean deposit;
    public boolean forceDown;
    public DistanceSensor distance;

    public DcMotorEx intake;

    //Used for intake method
    //ColorSensor color = hardwareMap.get(ColorSensor.class, "color");

    /*
     * Creates, declares, and assigns a motor to the motors array list
     */
    public ConciseIntake(DcMotorEx intake, Servo intake2, DistanceSensor d) {
        super();
        this.intake = intake;
        motors.add(intake);
        motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get(0).setDirection(DcMotorSimple.Direction.FORWARD);
        servos.add(intake2);
        distance = d;
    }
    public ConciseIntake(DcMotorEx intake, Servo intake2) {
        super();
        this.intake = intake;
        motors.add(intake);
        motors.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servos.add(intake2);
    }
    //Intakes for a given time in seconds
    public void intake(int duration) {
        Deadline time = new Deadline(duration, TimeUnit.SECONDS);
        while(!time.hasExpired()){
            intake.setVelocity(60); //Add number
            intake.setPower(60); //Add number
        }
    }

    //Deposits for a given time in seconds
    public void deposit(int duration) {
        Deadline time = new Deadline(duration, TimeUnit.SECONDS);
        while(!time.hasExpired()){
 //Add number
            intake.setPower(50); //Add number
        }
    }

    /*
     * Controls the robot during TeleOp and sends input to run
     */
    public void update(Gamepad gp1, Gamepad gp2) {
        intaking = gp1.right_trigger >= .3;
        outputting = gp1.left_trigger >= .3;
        deposit = gp1.left_bumper;

        if(gp1.right_trigger <= .3 && distance.getDistance(DistanceUnit.CM)>= .8){
            forceDown = false;
        } else if(gp1.a){
            forceDown = true;}
    }

     //Controls the intake
    public void write() {
        run();
        if ((forceDown || distance.getDistance(DistanceUnit.CM)<0.8) && !intaking) {
            servos.get(0).setPosition(0.4);
        } else {
            servos.get(0).setPosition(0.1);
        }
    }

     //Controls the intake during TeleOp using input from update
    public void run() {
        if (intaking) {
            motors.get(0).setPower(1);
            servos.get(0).setPosition(0.1);
        } else if (outputting) {
            motors.get(0).setPower(-1);
        } else if (deposit) {
            motors.get(0).setPower(-0.5);
            forceDown = false;
        } else {
            motors.get(0).setPower(0);
        }
    }
}

/*
    Pseudocode:
    When a button is pressed
    Turn of intake
    When another button is pressed
    Reverse intake
    (Optionally) Reverse intake when there is more than one cube in the robot
*/