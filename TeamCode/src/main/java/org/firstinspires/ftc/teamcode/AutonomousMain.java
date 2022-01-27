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
    private final Location levelOneGrab = new Location(24.97,0,345.29,358.16); // 50.92, 494.9, -7.97
    private final Location levelOneDeposit = new Location (640, 0, 549.9, 92.2);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        spinner = new Spinner(hardwareMap.get(CRServo.class,"spinner"), hardwareMap.get(Servo.class, "carouselB"));
        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry);
        cap = new Cap(hardwareMap.get(Servo.class, "capServo"));
        vuforia = new Vuforia();
        time = new ElapsedTime();

        robot.odometry.reset();
        drive.odoDown();

//        OpenCvCamera webcam;
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.openCameraDevice();
//        webcam.setPipeline(new Vuforia());
//        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//        vuforia = new Vuforia(webcam);

        while (!isStarted()) {
            robot.odometry.updatePosition();
            drive.telemetry.addData("Odometry", robot.odometry.getPosition().getLocation(0) + ", " + robot.odometry.getPosition().getLocation(2) + ", " + robot.odometry.getPosition().getLocation(3));
            mode = vuforia.getMode();
            telemetry.addData("Mode", mode);
            telemetry.addData("Area", vuforia.getArea());
            telemetry.update();

        }
        waitForStart();
//        switch(mode) {
//
//            case 2:
////                drive.moveToPosition(ZONE_B, 25, 25, 1, 2500);
//                break;
//            case 3:
////                drive.moveToPosition(ZONE_C, 25, 25, 1, 2500);
//                break;
//            default:
////                drive.moveToPosition(ZONE_A, 25, 25, 1, 2000);
//                break;
//        }
        cap.moveToIntakeHeight();
        time.reset();
        while(time.seconds()<3) {
            spinner.crServos.get(0).setPower(60);
            spinner.servos.get(0).setPosition(0.5);
        }
        spinner.crServos.get(0).setPower(0);
        drive.moveToPositionSlow(levelOneGrab,5, 5,2);
        transfer.motors.get(0).setPower(50);
        transfer.motors.get(0).setTargetPosition(-600);
        sleep(500);
        cap.moveToStoringHeight();
        sleep(2000);
//        drive.moveToPosition(levelOneDeposit,5,5,2);

    }
}