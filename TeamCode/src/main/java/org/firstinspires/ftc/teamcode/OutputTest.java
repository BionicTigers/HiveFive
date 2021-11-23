package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Output")
public class OutputTest extends LinearOpMode {

    public Output output;

    @Override
    public void runOpMode() throws InterruptedException {
        output = new Output(hardwareMap.get(Servo.class, "output"));

        waitForStart();

        while (opModeIsActive()) {
            output.update(gamepad1, gamepad2);
            output.write();
        }
    }
}