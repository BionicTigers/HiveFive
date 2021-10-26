package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class OutputTest extends LinearOpMode {

    public Output output;

    @Override
    public void runOpMode() throws InterruptedException {
            output = new Output((Servo) hardwareMap.get(Servo.class, "output"));

            output.update(gamepad1, gamepad2);
            output.write();
    }
}