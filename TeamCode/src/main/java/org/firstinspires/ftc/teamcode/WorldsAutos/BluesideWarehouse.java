package org.firstinspires.ftc.teamcode.WorldsAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Cap;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.EvilVision;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Location;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Spinner;
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.Vuforia;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

//Positions needed: Hub scoring 1/2/3, intermediate, warehouse
/*
Uses vision to scan barcode, then scores preload on appropriate level. Until a set time within the
auto, cycles as much freight as possible onto level 3, then parks in the warehouse.
*/
@Autonomous(name="Warehouse Blue Worlds")
public class BluesideWarehouse extends LinearOpMode {
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"}; //establishes motor names
    public Drivetrain drivetrain; //declares drivetrain
    public Robot robot; //declares robot
    public Intake intake;
    public Cap cap;
    public Spinner spinner;
    public Turret turret;
    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array
    Vuforia vuforia;

    private Location position = new Location();
    private int[] wheels = {0, 1, 2, 3};
    private int mode;

    DistanceSensor distance;


    public boolean hasFreight = false;

    private final Location dropZone = new Location(310,0,-200,0);
    private final Location wall = new Location(-10,0,0,0);
    private final Location warehouse = new Location(10,0,900,0);
    private final Location parky = new Location(500,0,900,0);

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
        turret.servos.get(0).setPosition(0.456);
        turret.servos.get(1).setPosition(0.60);
        drivetrain.odoDown();
        OpenCvCamera webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new Vuforia());
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        vuforia = new Vuforia(webcam);

        intake.servos.get(0).setPosition(.6);
        while(!isStarted() && !isStopRequested()){
            if(gamepad1.a) {
                turret.motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.motors.get(1).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            telemetry.addData("mode", mode);
            mode = vuforia.getMode();
            telemetry.addData("area", vuforia.getArea());
            telemetry.update();
        }
        waitForStart();
        turret.motors.get(0).setTargetPosition(0);
        turret.motors.get(1).setTargetPosition(0);
        turret.motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.motors.get(1).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.odometry.reset();
        turret.motors.get(1).setPower(100);
        turret.motors.get(0).setTargetPosition(-850);
        switch(mode) {
            case 1:
                turret.motors.get(1).setTargetPosition(-900);
                break;
            case 2:
                turret.motors.get(1).setTargetPosition(-2000);
                break;
            default:
                turret.motors.get(1).setTargetPosition(-2900);
                turret.motors.get(0).setTargetPosition(-625);
                break;
        }


        sleep(1000);
        turret.servos.get(0).setPosition(0.856);
        turret.servos.get(1).setPosition(0.2);
        turret.motors.get(0).setPower(75);
        drivetrain.moveToPosition(dropZone, 5, 5, 2,2000);
        turret.servos.get(0).setPosition(0.78);
        turret.servos.get(1).setPosition(0.276);
            //Vision stuffs

            intake.servos.get(0).setPosition(.1);
            intake.motors.get(0).setPower(-.65);
            sleep(500);
            turret.servos.get(0).setPosition(.5);
            turret.servos.get(1).setPosition(.556);
            sleep(1000);
            intake.motors.get(0).setPower(0);
            intake.servos.get(0).setPosition(0);
            turret.motors.get(0).setTargetPosition(2168);
            turret.motors.get(0).setPower(60);
            sleep(1000);
            turret.motors.get(1).setPower(60);
            turret.motors.get(1).setTargetPosition(-600);
            sleep(1000);
            drivetrain.moveToPosition(wall, 5, 5, 2,2000);
            turret.motors.get(1).setTargetPosition(0);
            turret.servos.get(0).setPosition(0.456);
            turret.servos.get(1).setPosition(0.6);
        drivetrain.moveToPosition(warehouse, 5, 5, 2,2000);
        drivetrain.moveToPosition(parky, 5, 5, 2, 2000);
        drivetrain.odoUp();
        sleep(1000);
        }
    }

//Depositing freight position
//465.544, 0, 0

//Just in the warehouse
//0,

