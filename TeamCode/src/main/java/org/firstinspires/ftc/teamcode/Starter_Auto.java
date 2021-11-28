package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Location;

@Autonomous(name="Starter Auto", group="Autonomous")
public class Starter_Auto extends LinearOpMode {
    private Robot robo;
    private Intake intaker;
    //private Drivetrain drivey;

    private int[] wheels = {0, 1, 2, 3};

    private final Location Tester = new Location(100, 0, 100, 100);

    @Override
    public void runOpMode() throws InterruptedException {

        robo = new Robot (this);
        //drivey = new Drivetrain(robo, wheels, telemetry);
        intaker = new Intake((DcMotorEx) hardwareMap.get(DcMotor.class, "intake"));


        waitForStart();
        intaker.inspin();
        sleep(2000);
        intaker.nospin();
        sleep(2000);
        intaker.outspin();
    }
}