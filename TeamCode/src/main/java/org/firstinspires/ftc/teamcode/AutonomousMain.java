package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name="Autonomous")
public class AutonomousMain extends LinearOpMode{
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

    private final Location ZONE_A = new Location(-400, 0, 1850, 270);
    private final Location ZONE_B = new Location(-950, 0, 2500.92f, 270);
    private final Location ZONE_C = new Location(-450, 0, 3150, 270);
    private final Location levelOneGrab = new Location (-532.583, 0, 331.488, 350.698);
    private final Location levelOneDeposit = new Location (-238.56, 0, 807.51, 34.6);
    private final Location levelTwoGrab = new Location (-234.75,0, 412.71, 356.56);
    private final Location levelTwoDeposit = new Location (-276.59, 0, 747.14, 31.01);
    private final Location levelThreeGrab = new Location(24.97,0,395.29,358.16);
    private final Location levelThreeDeposit = new Location (302.45, 0, 619.27, 83.90);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        spinner = new Spinner(hardwareMap.get(CRServo.class,"spinner"), hardwareMap.get(Servo.class, "carouselB"));
        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry);
        cap = new Cap(hardwareMap.get(Servo.class, "capServo"));
        output = new Output(hardwareMap.get(Servo.class, "output"));
        vuforia = new Vuforia();
        time = new ElapsedTime();

        robot.odometry.reset();
        drive.odoDown();

        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new Vuforia());
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        vuforia = new Vuforia(webcam);

        while (!isStarted()) {
            robot.odometry.updatePosition();
            drive.telemetry.addData("Odometry", robot.odometry.getPosition().getLocation(0) + ", " + robot.odometry.getPosition().getLocation(2) + ", " + robot.odometry.getPosition().getLocation(3));
            mode = vuforia.getMode();
            telemetry.addData("Mode", mode);
            telemetry.addData("Area", vuforia.getArea());
            telemetry.update();
            if (gamepad1.a) {
                robot.odometry.reset();
            }
        }
        waitForStart();

        cap.moveToIntakeHeight();
        time.reset();
        while(time.seconds()<3) {
            spinner.crServos.get(0).setPower(-60);
            spinner.servos.get(0).setPosition(0.5);
        }
        spinner.crServos.get(0).setPower(0);
        switch(mode) {
            case 2:
                drive.moveToPositionSlow(levelTwoGrab, 10, 10, 2, 3000);
                sleep(500);
                cap.moveToStoringHeight();
                drive.moveToPositionSlow(levelTwoDeposit, 5, 5, 2, 3000);
                transfer.motors.get(0).setTargetPosition(-1760);
                transfer.motors.get(0).setPower(50);
                break;
            case 3:
                drive.moveToPositionSlow(levelThreeGrab,10, 10,2, 3000);
                sleep(500);
                cap.moveToStoringHeight();
                drive.moveToPositionSlow(levelThreeDeposit, 5, 5, 2, 3000);
                transfer.motors.get(0).setTargetPosition(-2280);
                transfer.motors.get(0).setPower(50);
                sleep(2000);
                break;
            default:
                drive.moveToPositionSlow(levelOneGrab,10, 10,2, 3000);
                sleep(500);
                cap.moveToStoringHeight();
                drive.moveToPositionSlow(levelOneDeposit, 5, 5, 2, 3000);
                transfer.motors.get(0).setTargetPosition(-1245);
                transfer.motors.get(0).setPower(50);
                break;
        }
        sleep(2000);
        output.servos.get(0).setPosition(0.3);
        sleep(2000);

//        drive.moveToPosition(levelOneDeposit,5,5,2);

    }
}