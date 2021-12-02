package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Spinner")
public class SpinnerTest extends LinearOpMode {

    public Spinner spinner;

    @Override
    public void runOpMode() throws InterruptedException {
        spinner = new Spinner((DcMotorEx)hardwareMap.get(DcMotor.class, "speener"));

        waitForStart();

        while (opModeIsActive()) {
            spinner.update(gamepad1, gamepad2);
            spinner.write();
        }
    }
}