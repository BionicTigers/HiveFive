package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RedCarousel", group = "Autonomous")
public class RedCarousel extends LinearOpMode {
    private Robot robot;
    private Intake intake;
    private PositionalTransfer transfer;
    public Output output;
    private Drivetrain drive;
    private Spinner spinner;

    private int[] wheels = {0, 1, 2, 3};
    public void runOpMode() {
        robot = new Robot(this);
        drive = new Drivetrain(robot, wheels, telemetry);
        spinner = new Spinner(hardwareMap.get(DcMotorEx.class, "spinner"));
        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry, hardwareMap.get(DigitalChannel.class, "channel"));
        output = new Output(hardwareMap.get(Servo.class, "output"));
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        waitForStart();
        drive.motors.get(0).setPower(0.5);
        drive.motors.get(1).setPower(0.5);
        drive.motors.get(2).setPower(0.5);
        drive.motors.get(3).setPower(0.5);
        sleep(200);
        drive.motors.get(0).setPower(0.5);
        drive.motors.get(1).setPower(-0.5);
        drive.motors.get(2).setPower(0.5);
        drive.motors.get(3).setPower(-0.5);
        sleep(300);
        drive.motors.get(0).setPower(0);
        drive.motors.get(1).setPower(0);
        drive.motors.get(2).setPower(0);
        drive.motors.get(3).setPower(0);
        spinner.backSpin();
        sleep(2300);
        drive.motors.get(0).setPower(0.5);
        drive.motors.get(1).setPower(0.5);
        drive.motors.get(2).setPower(0.5);
        drive.motors.get(3).setPower(0.5);
        sleep(550);
        drive.motors.get(0).setPower(0.5);
        drive.motors.get(1).setPower(-0.5);
        drive.motors.get(2).setPower(0.5);
        drive.motors.get(3).setPower(-0.5);
        sleep(500);
        drive.motors.get(0).setPower(0);
        drive.motors.get(1).setPower(0);
        drive.motors.get(2).setPower(0);
        drive.motors.get(3).setPower(0);
    }
}
