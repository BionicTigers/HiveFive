package org.firstinspires.ftc.teamcode.RookieTraining;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "driveTest")
public class DriveTest extends LinearOpMode {
    public Robot robot;
    public TrainDrive drive;
    public void runOpMode(){
        drive = new TrainDrive(hardwareMap.get(DcMotorEx.class, "frontLeft"), hardwareMap.get(DcMotorEx.class, "frontRight"), hardwareMap.get(DcMotorEx.class, "backLeft"), hardwareMap.get(DcMotorEx.class, "backRight"));
        waitForStart();
        drive.update(gamepad1, gamepad2);
        drive.write();
    }
}
