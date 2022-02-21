package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Autonomous")
public class AutonomousState extends LinearOpMode{
    private Robot robot;
    private Intake intake;
    private PositionalTransfer transfer;
    private Cap cap;
    public Output output;
    private Drivetrain drive;
    private Variables variables;
    private Vuforia vuforia;
    private Spinner spinner;
    private ElapsedTime time;

    private Location position = new Location();
    private int[] wheels = {0, 1, 2, 3};
    private int mode;
//still need location for deposit level 2 and 3, drop duck off at 3 btw
    private final Location levelOneDeposit = new Location (0, 0, 0, 0);
    private final Location levelOneDeposit2 = new Location (-264, 0, -553.46, 360);
    private final Location LevelOneMid = new Location(0, 0, 70, 0);
    private final Location postDepositOne = new Location(-264, 0, -453.46, 360);
    private final Location LevelonepreDuck = new Location(-615.10, 0, -252.41, 170.48);
    private final Location LeveloneDuck = new Location(-554.65, 0, -515.28, 169.16);
    private final Location Duckcarousel = new Location(-1235.26, 0, -327.34, 358.61);


    private final Location levelTwoDeposit = new Location (0, 0,0 , 0);
    private final Location LevelTwoMid = new Location(0, 0, 0, 0);
    private final Location LeveltwopreDuck = new Location(-832.70, 0, -186.98, 177.25);
    private final Location LeveltwoDuck = new Location(-807.08, 0, -533.32, 176.90);

    private final Location LevelThreeMid = new Location(-200, 0, 125, 0);
    private final Location levelThreeDeposit = new Location (-1100, 0, 450, 0);
    private final Location LevelthreepreDuck = new Location(-1055.09, 0, -159.15, 182.32);
    private final Location LevelthreeDuck = new Location(-1061.99, 0, -510.39, 181.82);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        spinner = new Spinner(hardwareMap.get(CRServo.class,"spinner"), hardwareMap.get(Servo.class, "carouselB"));
        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry, hardwareMap.get(DigitalChannel.class, "channel"));
        cap = new Cap(hardwareMap.get(Servo.class, "capServo"));
        output = new Output(hardwareMap.get(Servo.class, "output"));
        vuforia = new Vuforia();
        time = new ElapsedTime();
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));

        robot.odometry.reset();
        drive.odoDown();

        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new Vuforia());
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        vuforia = new Vuforia(webcam);

        while (!isStarted()&& !isStopRequested()) {
            robot.odometry.updatePosition();
            drive.telemetry.addData("Odometry", robot.odometry.getPosition().getLocation(0) + ", " + robot.odometry.getPosition().getLocation(2) + ", " + robot.odometry.getPosition().getLocation(3));
            mode = vuforia.getMode();
            telemetry.addData("Mode", mode);
            telemetry.addData("Area", vuforia.getArea());
            telemetry.update();
            if (gamepad1.a) {
                robot.odometry.reset();
            }
            transfer.motors.get(0).setTargetPosition(600* 312/435);
            transfer.motors.get(0).setPower(50);
            cap.servos.get(0).setPosition(0.05);
        }
        waitForStart();
        robot.odometry.reset();
        time.reset();
        transfer.motors.get(0).setTargetPosition(0);
        switch(mode) {
            case 2:


                drive.moveToPositionSlow(levelTwoDeposit, 5, 5, 2, 3000);
                transfer.motors.get(0).setTargetPosition(1700* 312/435);
                transfer.motors.get(0).setPower(80);
                output.servos.get(0).setPosition(1);
                sleep(1000);
                drive.moveToPosition(LeveltwopreDuck, 5, 5, 2, 3000);
                sleep(1000);
                drive.moveToPosition(LeveltwoDuck, 5, 5, 2, 3000);

                break;
            case 3:
                //drive.moveToPositionSlow(levelThreeGrab,10, 10,2, 1500);
//                sleep(500);
//                cap.moveToStoringHeight();
//                drive.moveToPosition(LevelThreeMid, 5, 5, 2, 500);

                drive.moveToPositionSlow(levelThreeDeposit, 5, 5, 2,3000);
                transfer.motors.get(0).setTargetPosition(2460* 312/435);
                transfer.motors.get(0).setPower(80);
                sleep(1000);
                drive.moveToPosition(LevelthreepreDuck, 5, 5, 2, 3000);
                sleep(1000);
                drive.moveToPosition(LevelthreeDuck, 5, 5, 2, 3000);

                break;
            case 1:
//                drive.moveToPositionSlow(levelOnepreGrab,10, 10,2, 1000);
//                drive.moveToPositionSlow(levelOneGrab,10, 10,2, 1500);
//                sleep(500);
//                cap.moveToStoringHeight();

                drive.moveToPositionSlow(levelOneDeposit2, 5, 5, 2, 4000);
                transfer.motors.get(0).setTargetPosition(1300 * 312/435);
                transfer.motors.get(0).setPower(80);
                sleep(1000);
                output.servos.get(0).setPosition(1);
                sleep(1000);
                drive.moveToPositionSlow(LevelonepreDuck, 5, 5, 2, 3000);
                output.servos.get(0).setPosition(0);
                transfer.motors.get(0).setTargetPosition(0);
                transfer.motors.get(0).setPower(80);
                sleep(1000);
                intake.run(true, false);
                drive.moveToPositionSlow(LeveloneDuck, 5, 5, 2, 3000);
                intake.run(false, false);

                break;
        }
    }
}
