package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Yes Cap")
/*OpMode for the Cap class*/
public class CapOpMode extends LinearOpMode {

    /*Declares a new instance of Cap*/
    public Cap nocap;

    /*Runs the Cap op mode*/
    public void runOpMode() throws InterruptedException {

        nocap = new Cap(hardwareMap.get(CRServo.class, "cap1"), hardwareMap.get(Servo.class, "cap2"));

        waitForStart();

        while (opModeIsActive()) {
            nocap.update(gamepad1, gamepad2);
            nocap.write();
        }
    }
}