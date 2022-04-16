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

@Autonomous(name="Blue Carousel Worlds")
public class BlueCarouselWorlds extends LinearOpMode {
    public String[] motorNames = {"frontRight", "frontLeft", "backLeft", "backRight"}; //establishes motor names
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
    private final Location carousel = new Location(-100, 0, -600, 0);
    private final Location hubScore = new Location(-1300, 0, -200, 0);
    private final Location storageUnit = new Location(-650, 0, -600, 0);
    private final Location noMansLand = new Location(-1300, 0, -600, 0);
    public Location turn = new Location(380, 0, -380, 90);


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "SDriveL"), hardwareMap.get(Servo.class, "SDriveM"), hardwareMap.get(Servo.class, "SDriveR"));
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"), hardwareMap.get(Servo.class, "intake2"));
        spinner = new Spinner(hardwareMap.get(DcMotorEx.class, "spinner"));
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
        waitForStart();
        drivetrain.moveToPosition(turn, 5, 5, 2, 2000);
    }
}