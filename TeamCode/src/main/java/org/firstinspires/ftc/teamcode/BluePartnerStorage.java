//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.concurrent.TimeUnit;
//
//@Autonomous(name = "Blue Partner Storage", group = "Autonomous")
//public class BluePartnerStorage extends LinearOpMode{
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
//    private final Location postDropMove = new Location(-260,0,-350,0);
//    private final Location preCarousel = new Location(-1057.30,0,-188.99,38.03);
//    private final Location carousel = new Location(-1425.81,0,-200.70,39);
//    private final Location preDuck = new Location(-1534.05,0,-612,93);
//    private final Location duck = new Location(-885.22,0,-622,93);
//    private final Location storageUnit = new Location(0,0,0,0); //get position
//    private final Location partnerPreload = new Location(0,0,0,0); //get position
//    private final Location preHubDuck = new Location(-800,0,-400,360);
//
//    private final Location levelOneDeposit = new Location (-259.58,0,-531.44,0);
//
//    private final Location levelTwoDeposit = new Location (-308.83, 0,-530,356.14);
//
//    private final Location levelThreeDeposit = new Location (-302.02, 0, -428.28, 356.67);
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = new Robot(this);
//        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDriveL"), hardwareMap.get(Servo.class, "SDriveM"), hardwareMap.get(Servo.class, "SDriveR"));
//        spinner = new Spinner(hardwareMap.get(DcMotorEx.class,"spinner"), hardwareMap.get(Servo.class, "carouselB"));
//        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry, hardwareMap.get(DigitalChannel.class, "channel"));
//        cap = new Cap(hardwareMap.get(Servo.class, "capServo"));
//        output = new Output(hardwareMap.get(Servo.class, "output"));
//        vuforia = new Vuforia();
//        time = new ElapsedTime();
//        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
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
//        while (!isStarted()&& !isStopRequested()) {
//            robot.odometry.updatePosition();
//            drive.telemetry.addData("Odometry", robot.odometry.getPosition().getLocation(0) + ", " + robot.odometry.getPosition().getLocation(2) + ", " + robot.odometry.getPosition().getLocation(3));
//            mode = vuforia.getMode();
//            telemetry.addData("Mode", mode);
//            telemetry.addData("Area", vuforia.getArea());
//            telemetry.update();
//            if (gamepad1.a) {
//                robot.odometry.reset();
//            }
//            transfer.motors.get(0).setTargetPosition(600 * 223/312);
//            transfer.motors.get(0).setPower(50);
//            cap.servos.get(0).setPosition(0.1);
//            spinner.motors.get(0).setTargetPosition(0);
//            spinner.motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        waitForStart();
//        robot.odometry.reset();
//        time.reset();
//        transfer.motors.get(0).setTargetPosition(0);
//        switch(mode) {
//            case 2:
//                drive.moveToPositionSlow(levelTwoDeposit, 5, 5, 2, 2000);
//                transfer.motors.get(0).setTargetPosition(1475* 223/312);
//                break;
//            case 3:
//                drive.moveToPositionSlow(levelThreeDeposit, 5, 5, 2, 2000);
//                transfer.motors.get(0).setTargetPosition(2460 * 223/312);
//                break;
//            default:
//                drive.moveToPositionSlow(levelOneDeposit, 5, 5, 2, 2000);
//                transfer.motors.get(0).setTargetPosition(1200 * 223/312);
//                break;
//        }
//        transfer.motors.get(0).setPower(80);
//        sleep(1000);
//        output.servos.get(0).setPosition(1);
//        sleep(500);
//        //Freight has been dropped
//        drive.moveToPosition(postDropMove, 5,5,2,500);
//        output.servos.get(0).setPosition(0.7);
//        transfer.motors.get(0).setTargetPosition(0);
//        transfer.motors.get(0).setPower(80);
//        drive.moveToPositionSlow(partnerPreload, 5,5,2,1500);
//        while(color.red()/55.0 > 21.0){
//            intake.motors.get(0).setPower(1);
//        }
//        intake.motors.get(0).setPower(0);
//        drive.moveToPositionSlow(levelThreeDeposit,5,5,2,2000);
//        transfer.motors.get(0).setTargetPosition(2460 * 223/312);
//        transfer.motors.get(0).setPower(80);
//        sleep(1000);
//        output.servos.get(0).setPosition(1);
//        sleep(500);
//        drive.moveToPositionSlow(postDropMove,5,5,2,500);
//        output.servos.get(0).setPosition(0.7);
//        transfer.motors.get(0).setTargetPosition(0);
//        transfer.motors.get(0).setPower(80);
//        drive.moveToPosition(preCarousel,5,5,2,1000);
//        drive.moveToPosition(carousel,5,5,2,2500);
//        spinner.servos.get(0).setPosition(0.2);
//        spinner.motors.get(0).setTargetPosition(1700);
//        spinner.motors.get(0).setPower(0.32);
//        sleep(1900);
//        spinner.motors.get(0).setPower(0);
//        spinner.servos.get(0).setPosition(0.46);
//        drive.moveToPositionSlow(preDuck, 5, 5, 2, 2500);
//        intake.motors.get(0).setPower(1);
//        drive.moveToPositionSuperSlow(duck, 5, 5, 2, 2750);
//        transfer.motors.get(0).setTargetPosition(600 * 223/312);
//        transfer.motors.get(0).setPower(80);
//        sleep(500);
//        intake.motors.get(0).setPower(0);
//        drive.moveToPosition(preHubDuck,5,5,2,500);
//        drive.moveToPositionSlow(levelThreeDeposit,5,5,2,1500);
//        hasFreight = false;
//        transfer.motors.get(0).setTargetPosition(2460 * 223/312);
//        transfer.motors.get(0).setPower(80);
//        sleep(1000);
//        output.servos.get(0).setPosition(1);
//        sleep(500);
//        output.servos.get(0).setPosition(0.7);
//        sleep(1000);
//        drive.moveToPositionSlow(storageUnit,5,5,2,2000);
//}}
