package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake")
public class IntakeTest extends LinearOpMode {

    public Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap.get(Servo.class, "intakeLeft"), hardwareMap.get(Servo.class, "intakeRight"));

        waitForStart();

        while (opModeIsActive()) {
            intake.update(gamepad1, gamepad2);
            intake.write();
        }
    }
}