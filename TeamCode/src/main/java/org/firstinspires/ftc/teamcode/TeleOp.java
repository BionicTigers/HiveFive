package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;

/**
 * Controls the movements of the robot during the TeleOp
 */
public class TeleOp extends LinearOpMode {
    /**creates a new instance of Drivetrain*/
    public Drivetrain drivetrain;
    /**creates a new instance of Cap*/
    public Cap cap;
    /**creates a new instance of Intake*/
    public Intake intake;
    /**creates a new instance of Robot*/
    public Robot robot;
    /**creates a new instance of Spinner*/
    public Spinner spinner;
    /**creates a new instance of Transfer*/
    public Transfer transfer;
    /**creates a new instance of Output*/
    public Output output;

    //private FtcDashboard dashboard;
    /**creates a new instance of Telemetry*/
    private Telemetry dashboardTelemetry;

    private int wheels[] ={0, 1, 2, 3};
    private int servos[] ={0, 1, 2, 3};

    @Override
    public void runOpMode() throws InterruptedException {
        Variables.mult= (float) .3;
        robot = new Robot(this,Variables.transitionLocation);
        //dashboard = FtcDashboard.getInstance();
        //dashboardTelemetry = dashboard.getTelemetry();

        //Assigns servos to the intake object
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"), hardwareMap.get(Servo.class, "intakeLeft"), hardwareMap.get(Servo.class, "intakeRight"));
        //Adds a motor to the transfer object
        transfer = new Transfer((DcMotorEx) hardwareMap.get(DcMotor.class, "transfer"));
        //Assigns a servo to the output object
        output = new Output(hardwareMap.get(Servo.class, "output"));
        //Assigns motors to the drivetrain object
        drivetrain = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        //Assigns servos to the carousel spinner object
        spinner = new Spinner(hardwareMap.get(CRServo.class, "spinner"), hardwareMap.get(Servo.class, "carouselB"));
        //Assigns a servo to the cap arm object
        cap = new Cap(hardwareMap.get(Servo.class, "capServo"));

        //dtServo = new dtServo(hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        //Creates an array list with all the mechanisms in it
        Mechanism[] mechanisms ={intake, transfer, output, drivetrain, spinner, cap, robot.odometry};
        waitForStart(); //Doesn't progress until the start button is pressed

        while(opModeIsActive()) { //Runs through this code constantly after the start button is pressed

            telemetry.addData("Encoder positions: ", robot.odometry.currentEncoderPosString());
            for (Mechanism mech : mechanisms) { //For each mechanism in the mechanism array
                mech.update(gamepad1, gamepad2); //Run their respective update methods
            }

            for (Mechanism mech : mechanisms) { //For each mechanism in the mechanism array
                mech.write(); //Run their respective write methods
            }
            for (Mechanism mech : mechanisms) { //For each mechanism in the mechanism array
                ArrayList<String> captions = mech.getTelemetryCaptions();
                ArrayList<String> data = mech.getTelemetryDatas();
                int size = captions.size();
                //runs through the array adding the data as needed
                for(int i=0; i<size;i++){
                    telemetry.addData(captions.get(i), data.get(i));
                    dashboardTelemetry.addData(captions.get(i),data.get(i));
                }
                mech.updatePublicTelem();
            }


            //telemetry.addData("Robot position", robot.odometry.currentRobotPositionString());
            // telemetry.addData("distance", shooter.length);

            telemetry.update();
            dashboardTelemetry.update();
        }
        Variables.transitionLocation = robot.odometry.getPos();
    }
}
