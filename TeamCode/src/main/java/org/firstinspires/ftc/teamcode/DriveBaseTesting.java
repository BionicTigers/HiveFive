package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Drivetrain Test")
public class DriveBaseTesting extends LinearOpMode {
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"};
    public Robot robot;
    public Drivetrain drivetrain;
    public int[] motorNumbers = {0, 1, 2, 3};

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.update(gamepad1, gamepad2);
            drivetrain.write();
        }
    }
}
