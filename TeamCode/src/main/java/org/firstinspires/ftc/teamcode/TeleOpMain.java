package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/*
This class calls the different mechanisms on the robot so that they can all be used based on the
controller input
*/

@TeleOp (name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode{
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"}; //establishes motor names
    public Drivetrain drivetrain; //declares drivetrain
    public Robot robot; //declares robot
    public Intake intake;
    public PositionalTransfer transfer;
    public Output output;
    public Cap cap;
    public Spinner spinner;
    //public BNO055IMU gyro;

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array


    public void runOpMode() {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry);
        output = new Output(hardwareMap.get(Servo.class, "output"));
        spinner = new Spinner(hardwareMap.get(CRServo.class, "spinner"), hardwareMap.get(Servo.class, "carouselB"));
        cap = new Cap(hardwareMap.get(Servo.class, "capServo"));
        robot.initMotors(motorNames);
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        gyro = new Gyro(telemetry);
//        gyro = hardwareMap.get(BNO055IMU.class, "imu");
//        gyro.initialize(parameters);
        //These lines set motors and servos to their default position once teleOp starts
        waitForStart();

        output.servos.get(0).setPosition(.7);
        drivetrain.odoUp();

        Mechanism[] mechanisms = {intake, transfer, output, spinner, drivetrain, cap, robot.odometry};

        //what runs constantly once play button is pressed
        while(opModeIsActive()) {

            for (Mechanism mech : mechanisms) { //For each mechanism in the mechanism array
                mech.update(gamepad1, gamepad2); //Run their respective update methods
            }

            for (Mechanism mech : mechanisms) { //For each mechanism in the mechanism array
                mech.write(); //Run their respective write methods
            }
        }
    }
}
