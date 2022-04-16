package org.firstinspires.ftc.teamcode.WorldsAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Cap;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Location;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Spinner;
import org.firstinspires.ftc.teamcode.Turret;

import java.util.concurrent.TimeUnit;

@Autonomous(name="carousel Red Worlds")
public class RedCarouselWorlds extends LinearOpMode {
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"}; //establishes motor names
    public Drivetrain drivetrain; //declares drivetrain
    public Robot robot; //declares robot
    public Intake intake;
    public Cap cap;
    public Spinner spinner;
    public Turret turret;
    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    private Location position = new Location();
    private int[] wheels = {0, 1, 2, 3};
    private int mode;

    DistanceSensor distance;

    public boolean hasFreight = false;

    private final Location precarousel = new Location(-100, 0, 0, 0);
    private final Location carousel = new Location(-100,0,-600,0);
    private final Location hubScore = new Location(-1300,0,-200,0);
    private final Location storageUnit = new Location(-650,0,-600,0);
    private final Location noMansLand = new Location(-1300,0,-600,0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "SDriveL"), hardwareMap.get(Servo.class, "SDriveM"), hardwareMap.get(Servo.class, "SDriveR"));
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"), hardwareMap.get(Servo.class, "intake2"));
        spinner = new Spinner(hardwareMap.get(DcMotorEx.class,"spinner"));
        cap = new Cap(hardwareMap.get(CRServo.class, "cap1"), hardwareMap.get(Servo.class, "cap2"), telemetry);
        turret = new Turret(hardwareMap.get(DcMotorEx.class, "turretSpin"), hardwareMap.get(DcMotorEx.class, "turretLift"), hardwareMap.get(Servo.class, "turretLeft"), hardwareMap.get(Servo.class, "turretRight"), telemetry);
        robot.initMotors(motorNames);
        distance = hardwareMap.get(DistanceSensor.class, "distance");


        Deadline park = new Deadline(25, TimeUnit.SECONDS);
        cap.getServos().get(0).setPosition(0.5);
        turret.servos.get(0).setPosition(0.456);
        turret.servos.get(1).setPosition(0.60);
        drivetrain.odoDown();

        cap.getServos().get(0).setPosition(.5);

        intake.servos.get(0).setPosition(0.4);
        while(!isStarted() && !isStopRequested()){
            if(gamepad1.a) {
                turret.motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.motors.get(1).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            spinner.motors.get(0).setTargetPosition(0);
            spinner.motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        waitForStart();
        turret.servos.get(0).setPosition(0.656);
        turret.servos.get(1).setPosition(0.4);
        turret.motors.get(0).setTargetPosition(0);
        turret.motors.get(1).setTargetPosition(-400);
        turret.motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.motors.get(1).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.motors.get(1).setPower(100);
        turret.servos.get(0).setPosition(0.446);
        turret.servos.get(1).setPosition(0.61);
        robot.odometry.reset();
        drivetrain.moveToPositionSlow(precarousel, 5, 5, 2, 2000);
        drivetrain.moveToPositionSlow(carousel, 5, 5, 2, 2000);
        spinner.motors.get(0).setTargetPosition(-3100);
        spinner.motors.get(0).setPower(0.67);
        sleep(3600);
        spinner.motors.get(0).setPower(0);
        drivetrain.moveToPositionSlow(noMansLand, 5, 5, 2, 2000);
        drivetrain.moveToPosition(hubScore, 5, 5, 2, 2000);
        turret.motors.get(0).setTargetPosition(-2100);
        turret.motors.get(1).setTargetPosition(-2900);
        turret.motors.get(1).setPower(100);
        sleep(1000);
        turret.motors.get(0).setPower(100);
        sleep(1200);
        turret.servos.get(0).setPosition(0.856);
        turret.servos.get(1).setPosition(0.2);
        sleep(500);
        intake.servos.get(0).setPosition(.1);
        sleep(250);
        intake.motors.get(0).setPower(.35);
        sleep(1000);
        intake.motors.get(0).setPower(0);
        intake.servos.get(0).setPosition(0);
        sleep(200);
//        turret.motors.get(0).setTargetPosition(-2590);
//        turret.motors.get(1).setPower(60);
//        turret.motors.get(1).setTargetPosition(400);
//        turret.motors.get(1).setTargetPosition(0);
//        intake.motors.get(0).setPower(-100);
//        intake.servos.get(0).setPosition(0);
//        drivetrain.moveToPositionSlow(noMansLand, 5, 5, 2, 2000);
//        drivetrain.moveToPositionSlow(storageUnit, 5, 5, 2, 2000);
    }
}

