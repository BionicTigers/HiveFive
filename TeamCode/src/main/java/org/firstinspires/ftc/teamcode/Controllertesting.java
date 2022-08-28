package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Mechanism;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "ControllerTesting")
public class Controllertesting extends LinearOpMode {
    public Robot robot;
    public DriveTank drive;
    public void runOpMode(){
        drive = new DriveTank (hardwareMap.get(DcMotorEx.class, "frontLeft"), hardwareMap.get(DcMotorEx.class, "frontRight"), hardwareMap.get(DcMotorEx.class, "backLeft"), hardwareMap.get(DcMotorEx.class, "backRight"));
        waitForStart();
        while(opModeIsActive()) {
            drive.update(gamepad1, gamepad2);
            drive.write();
        }
    }
}