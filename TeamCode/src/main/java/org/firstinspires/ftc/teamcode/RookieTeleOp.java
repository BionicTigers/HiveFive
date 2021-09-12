package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot;

@TeleOp(name="ben_moment")
public class RookieTeleOp extends LinearOpMode {

    public RookieDrivetrain thomas;
    public Robot rookieBot = new org.firstinspires.ftc.teamcode.Mechanisms.Robot(this);
    public int[] motorNumbers = {0, 1, 2, 3};

    public void runOpMode() {
        thomas = new RookieDrivetrain(rookieBot, motorNumbers);


        //what runs constantly once play button is pressed
        while(opModeIsActive()) {
            thomas.update(gamepad1, gamepad2);
            thomas.write();
        }
    }
}
