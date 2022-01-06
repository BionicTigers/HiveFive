package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoStuff.Variables;

@TeleOp(name="Auto Return")
public class AutoReturn extends LinearOpMode {
    private Robot robot;
    private Intake intake;
    private Drivetrain drive;
    private Variables variables;

    private Location position = new Location();


    private int[] wheels = {0, 1, 2, 3};

    private final Location Away = new Location(0, 0, -700, 0);
    private final Location Back = new Location(0,0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(this);
        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"), hardwareMap.get(Servo.class, "intakeLeft"), hardwareMap.get(Servo.class, "intakeRight"), hardwareMap.get(Servo.class, "blocker"));

        robot.odometry.reset();
        drive.odoDown();
        intake.servos.get(0).setPosition(0.83);
        intake.servos.get(1).setPosition(0.7);
        intake.servos.get(2).setPosition(0.5);

        while (!isStarted()) {
            robot.odometry.updatePosition();
            drive.telemetry.addData("Odometry", robot.odometry.getPosition().getLocation(0) + ", " + robot.odometry.getPosition().getLocation(2) + ", " + robot.odometry.getPosition().getLocation(3));
            drive.telemetry.update();
        }

        waitForStart();
//        intake.inspin();
//        sleep(2000);
//        intake.nospin();
//        sleep(2000);
//        intake.outspin();
//        sleep(2000);
//        drive.motors.get(0).setPower(1);
//        drive.motors.get(1).setPower(1);
//        drive.motors.get(2).setPower(1);
//        drive.motors.get(3).setPower(1);
//        sleep(750);
//        drive.motors.get(0).setPower(0);
//        drive.motors.get(1).setPower(0);
//        drive.motors.get(2).setPower(0);
//        drive.motors.get(3).setPower(0);
        while (opModeIsActive()) {
            while (!isStopRequested() && gamepad1.a) {
                drive.moveToPosition(Away,5,5,0.5);
            }

            while (!isStopRequested() && gamepad1.b) {
                drive.moveToPosition(Back,5,5,0.5);
            }
            robot.odometry.update(gamepad1, gamepad2);
        }
//        drive.moveToPosition(Tester, 25, 25,15,20000);
        }
}