package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

//Just parks in red storage unit
@Autonomous(name = "Red Storage Park")
public class RedStoragePark extends LinearOpMode{
    private Robot robot;
    private Intake intake;
    private PositionalTransfer transfer;
    private Cap cap;
    public Output output;
    private Drivetrain drive;
    private Variables variables;
    private EvilVision vision;
    private Spinner spinner;
    private ElapsedTime time;
    public ColorSensor color;

    private Location position = new Location();
    private int[] wheels = {0, 1, 2, 3};

    private final Location storageUnit = new Location(0,0,0,0); //get position

    public void runOpMode(){
        robot = new Robot(this);
        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDriveL"), hardwareMap.get(Servo.class, "SDriveM"), hardwareMap.get(Servo.class, "SDriveR"));
        spinner = new Spinner(hardwareMap.get(DcMotorEx.class,"spinner"), hardwareMap.get(Servo.class, "carouselB"));
        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry, hardwareMap.get(DigitalChannel.class, "channel"));
        cap = new Cap(hardwareMap.get(Servo.class, "capServo"));
        output = new Output(hardwareMap.get(Servo.class, "output"));
        vision = new EvilVision();
        time = new ElapsedTime();
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        color = hardwareMap.get(ColorSensor.class, "color");
        Deadline stop = new Deadline(28, TimeUnit.SECONDS);
        Deadline rightTurn = new Deadline(1, TimeUnit.SECONDS);
        waitForStart();
        drive.moveToPositionSlow(storageUnit,5,5,2,4000);
    }
}

