package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Location;

@Autonomous(name="Starter Auto", group="Autonomous")
public class StarterAuto extends LinearOpMode {
    private Robot robot;
    private Intake intake;
    private Drivetrain drive;


    private Location position = new Location();


    private int[] wheels = {0, 1, 2, 3};

    private final Location Tester = new Location(100, 0, 100, 100);

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot (this);
        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
//        intake = new Intake((DcMotorEx) hardwareMap.get(DcMotor.class, "intake"));

        robot.odometry.reset();


        waitForStart();
//        intake.inspin();
//        sleep(2000);
//        intake.nospin();
//        sleep(2000);
//        intake.outspin();
//        sleep(2000);
        drive.actuallyMoveToPosition(Tester, 25, 25, 1, 2000);
    }
}