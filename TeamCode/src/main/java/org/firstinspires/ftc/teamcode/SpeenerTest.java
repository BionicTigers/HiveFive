package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Spinner")
public class SpeenerTest extends LinearOpMode {

    public Speener speener;

    @Override
    public void runOpMode() throws InterruptedException {
        speener = new Speener(hardwareMap.get(CRServo.class, "carouselA"), hardwareMap.get(Servo.class, "carouselB"));

        waitForStart();

        while (opModeIsActive()) {
            speener.update(gamepad1, gamepad2);
            speener.write();
        }
    }
}