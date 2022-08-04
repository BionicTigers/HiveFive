package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
import java.util.regex.Pattern;

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
    public ColorSensor color;

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    public void runOpMode() {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry);
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry, hardwareMap.get(DigitalChannel.class, "channel"));
        spinner = new Spinner(hardwareMap.get(DcMotorEx.class,"spinner"));
        output = new Output(hardwareMap.get(Servo.class, "output"));
        robot.initMotors(motorNames);

        //These lines set motors and servos to their default position once teleOp starts
        waitForStart();
        output.servos.get(0).setPosition(.7);
        Mechanism[] mechanisms = {intake, transfer, output, spinner, drivetrain};

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
