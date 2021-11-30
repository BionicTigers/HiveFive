package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Location;

@Autonomous(name="Starter Auto", group="Autonomous")
public class Starter_Auto extends LinearOpMode {
    private Robot robo;
    private Intake intaker;
    private Drivetrain drivey;


    private Location position = new Location();


    private int[] wheels = {0, 1, 2, 3};

    private final Location Tester = new Location(100, 0, 100, 100);

    @Override
    public void runOpMode() throws InterruptedException {

        robo = new Robot (this);
        drivey = new Drivetrain(robo, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
//        intaker = new Intake((DcMotorEx) hardwareMap.get(DcMotor.class, "intake"));

        robo.odometry.reset();


        waitForStart();
//        intaker.inspin();
//        sleep(2000);
//        intaker.nospin();
//        sleep(2000);
//        intaker.outspin();
//        sleep(2000);
        drivey.actuallyMoveToPosition(Tester, 25, 25, 1, 2000);
    }
}