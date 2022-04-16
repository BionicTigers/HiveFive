//package org.firstinspires.ftc.teamcode.Autos;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.internal.system.Deadline;
//import org.firstinspires.ftc.teamcode.AutoStuff.Variables;
//import org.firstinspires.ftc.teamcode.Cap;
//import org.firstinspires.ftc.teamcode.Drivetrain;
//import org.firstinspires.ftc.teamcode.Intake;
//import org.firstinspires.ftc.teamcode.Location;
//import org.firstinspires.ftc.teamcode.Output;
//import org.firstinspires.ftc.teamcode.PositionalTransfer;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.Spinner;
//import org.firstinspires.ftc.teamcode.Vuforia;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.concurrent.TimeUnit;
//
////OG auto
//@Autonomous(name="Blue Carousel")
//public class BlueCarousel extends LinearOpMode {
//    private Robot robot;
//    private Intake intake;
//    private PositionalTransfer transfer;
//    private Cap cap;
//    public Output output;
//    private Drivetrain drive;
//    private Variables variables;
//    private Vuforia vuforia;
//    private Spinner spinner;
//    private ElapsedTime time;
//    public ColorSensor color;
//
//    private Location position = new Location();
//    private int[] wheels = {0, 1, 2, 3};
//    private int mode;
//    public boolean hasFreight;
//    //still need location for deposit level 2 and 3, drop duck off at 3 btw
//    private final Location carousel = new Location(-51.45, 0, -8.95, 359.39);
//    // X=-51.45 Z=-8.95 R=359.39  (move to carousel)
//
//    private final Location Intermediate = new Location(800, 0, -230.36, 351.80);
//// (move to intermed pre score) X=852.37 Z=-230.36 R=351.80
//
//    private final Location levelOneDeposit = new Location(896.79, 0, -531.44, 353.34);
//// (1st level score) X=913.89 Z=-511.06 R=353.34
//
//    private final Location levelTwoDeposit = new Location(917.07, 0, -530, 352.95);
//// 2nd level score) X=917.07 Z=-548.03 R=352.95
//
//    private final Location levelThreeDeposit = new Location(896.79, 0, -428.28, 348.17);
//    //(3rd level score) X=896.79 Z=-437.73 R= 348.17
//
//    private final Location StorageUnit = new Location(-319.01, 0, -720.0, 85);
//// (Parking storage unit) X=-355.21 -644.76 R=95.43
//private final Location Postcube = new Location(211.44, 0, -130.3, 0);
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robot = new Robot(this);
//        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDriveL"), hardwareMap.get(Servo.class, "SDriveM"), hardwareMap.get(Servo.class, "SDriveR"));
//        spinner = new Spinner(hardwareMap.get(DcMotorEx.class,"spinner"));
////        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry, hardwareMap.get(DigitalChannel.class, "channel"), hardwareMap.get(Servo.class, "STransfer1"), hardwareMap.get(Servo.class, "STransfer2"));
////        cap = new Cap(hardwareMap.get(CRServo.class, "cap1"), hardwareMap.get(Servo.class, "cap2"));
//        output = new Output(hardwareMap.get(Servo.class, "output"));
//        vuforia = new Vuforia();
//        time = new ElapsedTime();
//        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"), hardwareMap.get(Servo.class, "intake2"));
//        color = hardwareMap.get(ColorSensor.class, "color");
//        Deadline stop = new Deadline(28, TimeUnit.SECONDS);
//        Deadline rightTurn = new Deadline(1, TimeUnit.SECONDS);
//
//        robot.odometry.reset();
//        drive.odoDown();
//
//        OpenCvCamera webcam;
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.openCameraDevice();
//        webcam.setPipeline(new Vuforia());
//        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//        vuforia = new Vuforia(webcam);
//
//        while (!isStarted() && !isStopRequested()) {
//            robot.odometry.updatePosition();
//            drive.telemetry.addData("Odometry", robot.odometry.getPosition().getLocation(0) + ", " + robot.odometry.getPosition().getLocation(2) + ", " + robot.odometry.getPosition().getLocation(3));
//            mode = vuforia.getMode();
//            telemetry.addData("Mode", mode);
//            telemetry.addData("Area", vuforia.getArea());
//            telemetry.update();
//            if (gamepad1.a) {
//                robot.odometry.reset();
//            }
//            transfer.motors.get(0).setTargetPosition(175);
//            transfer.motors.get(0).setPower(50);
//            spinner.motors.get(0).setTargetPosition(0);
//            spinner.motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        waitForStart();
//        cap.servos.get(0).setPosition(0.1);
//        robot.odometry.reset();
//        time.reset();
//        transfer.motors.get(0).setTargetPosition(0);
//
//        drive.moveToPosition(carousel, 5, 5, 2, 1000);
//        spinner.servos.get(0).setPosition(0.13);
//        spinner.motors.get(0).setTargetPosition(1700);
//        spinner.motors.get(0).setPower(0.32*4/3);
//        sleep(1900);
//        spinner.servos.get(0).setPosition(0.5);
//        drive.moveToPosition(Intermediate, 5, 5, 2, 2500);
//        transfer.motors.get(0).setPower(80);
//
//
//
//        switch (mode) {
//            case 2:
//                transfer.motors.get(0).setTargetPosition(1475 * 223 / 312);
//                drive.moveToPositionSlow(levelTwoDeposit, 5, 5, 2, 1500);
//                break;
//            case 3:
//                transfer.motors.get(0).setTargetPosition(2460 * 223 / 312);
//
//                drive.moveToPositionSlow(levelThreeDeposit, 5, 5, 2, 1500);
//                break;
//            case 1:
//                transfer.motors.get(0).setTargetPosition(1200 * 223 / 312);
//                drive.moveToPositionSlow(levelOneDeposit, 5, 5, 2, 1500);
//                break;
//        }
//        output.servos.get(0).setPosition(1);
//        sleep(600);
//        output.servos.get(0).setPosition(0.7);
//        drive.moveToPositionSlow(Intermediate, 5, 5, 2, 2500);
//        drive.moveToPositionSlow(Postcube, 5, 5, 2, 2500);
//        transfer.motors.get(0).setTargetPosition(0);
//        drive.moveToPositionSlow(StorageUnit, 5, 5, 62, 2500);
//
//    }
//}