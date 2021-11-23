package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Spinner")
public class SpeenerTest extends LinearOpMode {

    public Speener speener;

    @Override
    public void runOpMode() throws InterruptedException {
        speener = new Speener((DcMotorEx)hardwareMap.get(DcMotor.class, "speener"));

        waitForStart();

        while (opModeIsActive()) {
            speener.update(gamepad1, gamepad2);
            speener.write();
        }
    }
}