package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "WEEWOOWEEWOO")
public class QuickTest extends LinearOpMode {
    public TwoTest wow;


    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"};
    public Robot robot;
    public Drivetrain drivetrain;
    public int[] motorNumbers = {0, 1, 2, 3};

    public void runOpMode() throws InterruptedException {
        wow = new TwoTest(hardwareMap.get(DcMotorEx.class, "motor"), hardwareMap.get(DcMotorEx.class, "motor2"));


        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry);

        waitForStart();

        while(opModeIsActive()) {
            wow.update(gamepad1, gamepad2);


            drivetrain.update(gamepad1, gamepad2);
            drivetrain.write();
        }
    }
}
