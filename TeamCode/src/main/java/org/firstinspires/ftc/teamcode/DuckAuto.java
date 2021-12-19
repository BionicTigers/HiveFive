package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;

@Autonomous (name = "duckAuto", group = "autonomous")
public class DuckAuto extends LinearOpMode{
    public Robot robot = new Robot();
    public PositionalTransfer transfer;
    public Output output;
    public Drivetrain drivetrain;
    public Spinner spinner;
    public Cap cap;
    public Telemetry telemetry;
    //Position finding
    public Location location;
    public Odometry odometry;
    public Variables.Level lev;

    //Arrays
    /**Declares an array of wheels*/
    public static int wheels[] = {0, 1, 2, 3};

    @Override
    public void runOpMode() throws InterruptedException {
        //Field value assignments
        Variables.mult= (float) .3;
        //dashboard = FtcDashboard.getInstance();
        //dashboardTelemetry = dashboard.getTelemetry();

        //Assigns a motor to the transfer object
        transfer = new PositionalTransfer((DcMotorEx) hardwareMap.get(DcMotor.class, "transfer"), telemetry);
        //Assigns a servo to the output object
        output = new Output(hardwareMap.get(Servo.class, "output"));
        //Assigns motors to the drivetrain object
        drivetrain = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        //Assigns servos to the carousel spinner object
        spinner = new Spinner(hardwareMap.get(CRServo.class, "spinner"), hardwareMap.get(Servo.class, "carouselB"));
        //Assigns a servo to the cap arm object
        cap = new Cap(hardwareMap.get(Servo.class, "capServo"));

        //Actual auto
        //Start
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        //Scan barcode
        lev = Variables.Level.MIDDLE;//(When camera works) lev = vision.readBarcode();
        //Pick up team shipping element
        cap.moveToIntakeHeight();
        sleep(3000);
        cap.moveToStoringHeight();
        sleep(1000);
        //Drive to shipping hub
        //location =...
        drivetrain.actuallyMoveToPosition(location, 0.1, 0.1, 0.1, 5); //Change info
        sleep(5000);
        //Put freight on correct level
        switch (lev){
            case BOTTOM:
                transfer.moveToBottom();
                break;
            case MIDDLE:
                transfer.moveToMiddle();
                break;
            case TOP:
                transfer.moveToTop();
                break;
        }
        sleep(3000);
        output.deposit();
        sleep(500);
        //Drive to carousel
        //location = ...
        drivetrain.actuallyMoveToPosition(location, 0.1, 0.1, 0.1, 5); //Change info
        sleep(3000);
        //Move spinner to correct position
        spinner.moveArmOut(hardwareMap.get(Servo.class, "carouselB"));
        sleep(500);
        //Spin carousel
        spinner.spin(hardwareMap.get(Servo.class, "spinner"), 3);
        sleep(2000);
        //Move spinner back
        spinner.moveArmBack(hardwareMap.get(Servo.class, "carouselB"));
        //Move to storage unit?
        //location = ...
        drivetrain.actuallyMoveToPosition(location, 0.1, 0.1, 0.1, 5); //Change info
        sleep(500);
    }
}
