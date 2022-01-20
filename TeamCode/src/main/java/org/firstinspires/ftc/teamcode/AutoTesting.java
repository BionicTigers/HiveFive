package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoStuff.Variables;

@TeleOp(name="Auto Testing")
public class AutoTesting extends LinearOpMode {
    private Robot robot;
    private Intake intake;
    private Drivetrain drive;
    private Variables variables;

    private Location position = new Location();


    private int[] wheels = {0, 1, 2, 3};

    private final Location Tester = new Location(100f, 0, 0f, 0);
    private final Location Origin = new Location(0,0,0,0);
    private final Location Trun = new Location(variables.xTester, 0,variables.zTester,variables.rotationTester);

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(this);
        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));

        robot.odometry.reset();
        drive.odoDown();

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
                drive.moveToPosition(Trun,5,5,2);
            }
            robot.odometry.update(gamepad1, gamepad2);
        }
//        drive.moveToPosition(Tester, 25, 25,15,20000);
        }
}