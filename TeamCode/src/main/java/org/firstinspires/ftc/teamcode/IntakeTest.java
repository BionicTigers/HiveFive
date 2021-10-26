package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class IntakeTest extends LinearOpMode {

    public Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake((DcMotorEx) hardwareMap.get(DcMotor.class, "intake"));

        intake.update(gamepad1, gamepad2);
        intake.write();
    }
}