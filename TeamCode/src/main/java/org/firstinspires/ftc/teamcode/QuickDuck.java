package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Quick Duck")
public class QuickDuck extends LinearOpMode {
    private Robot robot;
    private Intake intake;
    private Drivetrain drive;
    private Spinner spinner;
    private ElapsedTime time;

    private Location position = new Location();
    private int[] wheels = {0, 1, 2, 3};

    private final Location Carousel = new Location(50,0,0,0);
    private final Location levelOneGrab = new Location(-41.3,0,-314.8,117.5);
    private final Location levelOneDeposit = new Location (630, 0, -549.9, 92.2);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        spinner = new Spinner(hardwareMap.get(CRServo.class,"spinner"), hardwareMap.get(Servo.class, "carouselB"));
        time = new ElapsedTime();

        while (!isStarted()) {
            robot.odometry.updatePosition();
            drive.telemetry.addData("Odometry", robot.odometry.getPosition().getLocation(0) + ", " + robot.odometry.getPosition().getLocation(2) + ", " + robot.odometry.getPosition().getLocation(3));
            drive.telemetry.update();
        }

        waitForStart();
        time.reset();
        spinner.moveArmOut();
        while(time.seconds()<3) {
            spinner.crServos.get(0).setPower(1);
        }
        spinner.crServos.get(0).setPower(0);
        drive.moveToPosition(levelOneGrab,5, 5,2);
        wait(5000);
        drive.moveToPosition(levelOneDeposit,5,5,2);
    }
}
