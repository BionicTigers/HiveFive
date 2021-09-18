package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="ben_moment")
public class RookieTeleOp extends LinearOpMode {
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"};
    public RookieDrivetrain thomas;
    public Robot rookieBot;

    public int[] motorNumbers = {0, 1, 2, 3};

    public void runOpMode() {
        rookieBot = new Robot(this);
        thomas = new RookieDrivetrain(rookieBot, motorNumbers);
        rookieBot.initMotors(motorNames);
        waitForStart();
        //what runs constantly once play button is pressed
        while(opModeIsActive()) {
            thomas.update(gamepad1, gamepad2);
            thomas.write();
        }
    }
}
