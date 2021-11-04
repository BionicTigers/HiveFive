package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*This class utilizes the drivetrain class in order to move the robot*/
@TeleOp (name = "drivetrain_testing")
public class DrivetrainTesting extends LinearOpMode {
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"}; //establishes motor names
    public Drivetrain drivetrain; //declares drivetrain
    public Robot robot; //declares robot

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array

    public void runOpMode() {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry);
        robot.initMotors(motorNames);
        waitForStart();

        //what runs constantly once play button is pressed
        while(opModeIsActive()) {
            drivetrain.update(gamepad1, gamepad2);
            drivetrain.write();
        }
    }
}
