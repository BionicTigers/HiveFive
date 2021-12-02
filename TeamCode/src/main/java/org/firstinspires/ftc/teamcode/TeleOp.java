package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;

/**
 *
 * controls the movements of the robot
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

        //Assigns a motor to the intake object
        intake = new Intake((DcMotorEx) hardwareMap.get(DcMotor.class, "intake"));
        //Adds a servo to the transfer object
        transfer = new Transfer((DcMotorEx) hardwareMap.get(DcMotor.class, "transfer"));
        //Assigns motors to the drivetrain object
        drivetrain = new Drivetrain(robot, wheels, telemetry,(Servo) hardwareMap.get(Servo.class, "odo1"),(Servo) hardwareMap.get(Servo.class, "odo2"),(Servo) hardwareMap.get(Servo.class, "odo3"));

        //Creates an array list with all the mechanisms in it
        Mechanism[] mechanisms ={intake, transfer, drivetrain, robot.odometry};
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


            //telemetry.addData("Robot position", odometry.currentRobotPositionString());
            // telemetry.addData("distance", shooter.length);

            telemetry.update();
            dashboardTelemetry.update();
        }
        Variables.transitionLocation = robot.odometry.getPos();
    }
}
