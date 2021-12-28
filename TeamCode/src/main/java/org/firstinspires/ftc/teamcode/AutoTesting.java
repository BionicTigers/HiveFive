package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Auto Testing", group="Autonomous")
public class AutoTesting extends LinearOpMode {
    private Robot robot;
    private Intake intake;
    private Drivetrain drive;


    private Location position = new Location();


    private int[] wheels = {0, 1, 2, 3};

    private final Location Tester = new Location(100f, 0, 0f, 100);

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot (this);
        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"), hardwareMap.get(Servo.class, "intakeLeft"), hardwareMap.get(Servo.class, "intakeRight"), hardwareMap.get(Servo.class, "blocker"));

        robot.odometry.reset();
        drive.odoDown();
        intake.servos.get(0).setPosition(0.83);
        intake.servos.get(1).setPosition(0.7);
        intake.servos.get(2).setPosition(0.5);

        while (!isStarted() && !isStopRequested()) {
        robot.odometry.updatePosition();
        drive.telemetree.addData("Odometry ", robot.odometry.realMaybe.getLocation(0) + ", " + robot.odometry.realMaybe.getLocation(2) + ", " + robot.odometry.realMaybe.getLocation(3));
        drive.telemetree.update();
        }
        waitForStart();
//        intake.inspin();
//        sleep(2000);
//        intake.nospin();
//        sleep(2000);
//        intake.outspin();
//        sleep(2000);
//        drive.motors.get(0).setPower(1);
//        drive.motors.get(1).setPower(1);
//        drive.motors.get(2).setPower(1);
//        drive.motors.get(3).setPower(1);
//        sleep(750);
//        drive.motors.get(0).setPower(0);
//        drive.motors.get(1).setPower(0);
//        drive.motors.get(2).setPower(0);
//        drive.motors.get(3).setPower(0);
        drive.actuallyMoveToPositionCorrect(Tester, 25, 25,15,20000);
    }
}