package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;
import org.firstinspires.ftc.teamcode.Cap;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.EvilVision;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Location;
import org.firstinspires.ftc.teamcode.Output;
import org.firstinspires.ftc.teamcode.PositionalTransfer;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Spinner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;


@Autonomous(name="Red Carousel")
public class RedCarousel extends LinearOpMode {
    private Robot robot;
    private Intake intake;
    private PositionalTransfer transfer;
    private Cap cap;
    public Output output;
    private Drivetrain drive;
    private Variables variables;
    private EvilVision evilvision;
    private Spinner spinner;
    private ElapsedTime time;
    public ColorSensor color;

    private Location position = new Location();
    private int[] wheels = {0, 1, 2, 3};
    private int mode;
    public boolean hasFreight;
    //still need location for deposit level 2 and 3, drop duck off at 3 btw
    private final Location carousel = new Location(800, 0, -104.16, 142.7);
// (Carousel) X=

    private final Location levelOneDeposit = new Location(-308.37, 0, -565, 0);
// (1st level score) X=


    private final Location levelTwoDeposit = new Location(-308.37, 0, -555, 0);
// 2nd level score) X=
    private final Location levelThreeDeposit = new Location(-308.37, 0, -425, 0);
    //(3rd level score) X=b
    private final Location StorageUnit = new Location(920, 0, -740, 87.59);
// (Parking storage unit) X=
private final Location Intermediate = new Location(-308.37, 0, -365, 350.17);


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(this);
        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDriveL"), hardwareMap.get(Servo.class, "SDriveM"), hardwareMap.get(Servo.class, "SDriveR"));
        spinner = new Spinner(hardwareMap.get(DcMotorEx.class,"spinner"));
        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry, hardwareMap.get(DigitalChannel.class, "channel"), hardwareMap.get(Servo.class, "STransfer1"), hardwareMap.get(Servo.class, "STransfer2"));
        cap = new Cap(hardwareMap.get(CRServo.class, "cap1"), hardwareMap.get(Servo.class, "cap2"));
        output = new Output(hardwareMap.get(Servo.class, "output"));
        evilvision = new EvilVision();
        time = new ElapsedTime();
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        color = hardwareMap.get(ColorSensor.class, "color");
        Deadline stop = new Deadline(28, TimeUnit.SECONDS);
        Deadline rightTurn = new Deadline(1, TimeUnit.SECONDS);

        robot.odometry.reset();
        drive.odoDown();

        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new EvilVision());
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        evilvision = new EvilVision(webcam);

        while (!isStarted() && !isStopRequested()) {
            robot.odometry.updatePosition();
            drive.telemetry.addData("Odometry", robot.odometry.getPosition().getLocation(0) + ", " + robot.odometry.getPosition().getLocation(2) + ", " + robot.odometry.getPosition().getLocation(3));
            mode = evilvision.getMode();
            telemetry.addData("Mode", mode);
            telemetry.addData("Area", evilvision.getArea());
            telemetry.update();
            if (gamepad1.a) {
                robot.odometry.reset();
            }
            transfer.motors.get(0).setTargetPosition(175);
            transfer.motors.get(0).setPower(50);
            spinner.motors.get(0).setTargetPosition(0);
            spinner.motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        waitForStart();
        cap.servos.get(0).setPosition(0.1);
        robot.odometry.reset();
        time.reset();
        transfer.motors.get(0).setTargetPosition(0);
        switch (mode) {
            case 2:
                drive.moveToPositionSlow(levelTwoDeposit, 5, 5, 2, 2000);
                transfer.motors.get(0).setTargetPosition(1475 * 223 / 312);
                break;
            case 3:
                drive.moveToPositionSlow(levelThreeDeposit, 5, 5, 2, 2000);
                transfer.motors.get(0).setTargetPosition(2460 * 223 / 312);
                break;
            default:
                drive.moveToPositionSlow(levelOneDeposit, 5, 5, 2, 2000);
                transfer.motors.get(0).setTargetPosition(1200 * 223 / 312);
                break;
        }
        transfer.motors.get(0).setPower(80);
        sleep(1000);
        output.servos.get(0).setPosition(1);
        sleep(500);
        transfer.motors.get(0).setTargetPosition(0);
        drive.moveToPositionSlow(Intermediate, 5, 5, 2, 2500);
        output.servos.get(0).setPosition(0.7);
        drive.moveToPositionSlow(carousel, 5, 5, 2, 3000);
        spinner.servos.get(0).setPosition(0);
        spinner.motors.get(0).setTargetPosition(-1700);
        spinner.motors.get(0).setPower(0.32);
        sleep(2200);
        spinner.servos.get(0).setPosition(0.5);
        sleep(500);

        drive.moveToPosition(StorageUnit, 5, 5, 2);

    }
}