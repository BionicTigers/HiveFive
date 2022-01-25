//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import java.util.ArrayList;
//import java.util.List;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.AutoStuff.Variables;
//import org.firstinspires.ftc.teamcode.Vuforia;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//@Autonomous(name="Vuforia Testing")
//public class VuforiaAutoTesting extends LinearOpMode{
//    private Robot robot;
//    private Intake intake;
//    private Drivetrain drive;
//    private Variables variables;
//    private Vuforia vuforia;
//
//    private Location position = new Location();
//    private int[] wheels = {0, 1, 2, 3};
//
//    private final Location ZONE_A = new Location(-400, 0, 1850, 270);
//    private final Location ZONE_B = new Location(-950, 0, 2500.92f, 270);
//    private final Location ZONE_C = new Location(-450, 0, 3150, 270);
//    private int mode;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = new Robot(this);
//        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
//        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"), hardwareMap.get(Servo.class, "intakeLeft"), hardwareMap.get(Servo.class, "intakeRight"), hardwareMap.get(Servo.class, "blocker"));
//        robot.odometry.reset();
//        drive.odoDown();
//        intake.servos.get(0).setPosition(0.83);
//        intake.servos.get(1).setPosition(0.7);
//        intake.servos.get(2).setPosition(0.5);
//        OpenCvCamera webcam;
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.openCameraDevice();
//        webcam.setPipeline(new Vuforia());
//        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//        vuforia = new Vuforia(webcam);
//
//        while (!isStarted()) {
//            robot.odometry.updatePosition();
//            drive.telemetry.addData("Odometry", robot.odometry.getPosition().getLocation(0) + ", " + robot.odometry.getPosition().getLocation(2) + ", " + robot.odometry.getPosition().getLocation(3));
//            drive.telemetry.update();
//            mode = Vuforia.getMode();
//            telemetry.addData("Mode", mode);
//            telemetry.addData("Mode", mode);
//            telemetry.addData("Area", vuforia.getArea());
//            telemetry.update();
//
//        }
//        waitForStart();
//
//
//        switch(mode) {
//
//            case 2:
//                drive.moveToPosition(ZONE_B, 25, 25, 1, 2500);
//                break;
//            case 3:
//                drive.moveToPosition(new Location(-450, 0, ZONE_C.getLocation(2), robot.odometry.getPos().getLocation(3)), 25, 25, 1, 1000);
//                drive.moveToPosition(ZONE_C, 25, 25, 1, 2500);
//                break;
//            default:
//                drive.moveToPosition(ZONE_A, 25, 25, 1, 2000);
//                break;
//        }
//
//
//    }
//}