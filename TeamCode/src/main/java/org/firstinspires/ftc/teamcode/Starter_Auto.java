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


    private final Location Tester = new Location(100, 0, 100, 100);

    @Override
    public void runOpMode() throws InterruptedException {

        robo = new Robot (this);
        intaker = new Intake((DcMotorEx) hardwareMap.get(DcMotor.class, "intake"));


        waitForStart();

    }
}