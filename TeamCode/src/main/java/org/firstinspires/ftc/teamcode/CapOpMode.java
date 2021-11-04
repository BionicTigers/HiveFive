package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Yes Cap")
public class CapOpMode extends LinearOpMode {

    public Cap nocap;

    public void runOpMode() throws InterruptedException {

        nocap = new Cap (hardwareMap.get(Servo.class,"capArm"));

        waitForStart();

        while (opModeIsActive()) {
            nocap.update(gamepad1, gamepad2);
            nocap.write();
        }
    }
}