//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.AutoStuff.Variables;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//@Autonomous(name="Evan's Eccentric BS")
//public class EvansEccentricBS extends LinearOpMode{
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
//
//    private Location position = new Location();
//    private int[] wheels = {0, 1, 2, 3};
//    private int mode;
//
//    private final Location levelOnepreGrab = new Location (-490, 0, 180, 0);
//    private final Location levelOneGrab = new Location (-490, 0, 470, 0);
//    private final Location levelOneDeposit = new Location (-1000, 0, -10, 0);
//    private final Location levelOneDeposit2 = new Location (-1000, 0, 165, 0);
//    private final Location LevelOneMid = new Location(-500, 0, 70, 0);
//
//    private final Location levelTwopreGrab = new Location (-270,0, 200.71, 0);
//    private final Location levelTwoGrab = new Location (-270,0, 450, 0);
//    private final Location levelTwoDeposit = new Location (-1100, 0,120 , 0);
//    private final Location LevelTwoMid = new Location(-500, 0, 100, 0);
//
//    private final Location levelThreeGrab = new Location(50,0,425,0);
//    private final Location LevelThreeMid = new Location(-200, 0, 125, 0);
//    private final Location levelThreeDeposit = new Location (-1000, 0, 75, 0);
//
//    private final Location SpecialBoyTurn = new Location (0, 0, 0, 270);
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = new Robot(this);
//        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
//        spinner = new Spinner(hardwareMap.get(CRServo.class,"spinner"), hardwareMap.get(Servo.class, "carouselB"));
//        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry);
//        cap = new Cap(hardwareMap.get(Servo.class, "capServo"));
//        output = new Output(hardwareMap.get(Servo.class, "output"));
//        vuforia = new Vuforia();
//        time = new ElapsedTime();
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
//        while (!isStarted()) {
//            robot.odometry.updatePosition();
//            drive.telemetry.addData("Odometry", robot.odometry.getPosition().getLocation(0) + ", " + robot.odometry.getPosition().getLocation(2) + ", " + robot.odometry.getPosition().getLocation(3));
//            mode = vuforia.getMode();
//            telemetry.addData("Mode", mode);
//            telemetry.addData("Area", vuforia.getArea());
//            telemetry.update();
//            if (gamepad1.a) {
//                robot.odometry.reset();
//            }
//        }
//        waitForStart();
//        cap.moveToIntakeHeight();
//        time.reset();
//        while(time.seconds()<3) {
//            spinner.crServos.get(0).setPower(-60);
//            spinner.servos.get(0).setPosition(0.5);
//        }
//        spinner.crServos.get(0).setPower(0);
//        switch(mode) {
//            case 2:
//                drive.moveToPositionSlow(levelTwopreGrab, 10, 10, 2, 1000);
//                drive.moveToPositionSlow(levelTwoGrab, 10, 10, 2, 1500);
//                sleep(500);
//                cap.moveToStoringHeight();
//                drive.moveToPosition(LevelTwoMid, 5, 5, 2, 500);
//                drive.moveToPositionSlow(levelTwoDeposit, 5, 5, 2, 3000);
//                transfer.motors.get(0).setTargetPosition(-1700);
//                transfer.motors.get(0).setPower(80);
//                break;
//            case 3:
//                drive.moveToPositionSlow(levelThreeGrab,10, 10,2, 1500);
//                sleep(500);
//                cap.moveToStoringHeight();
//                drive.moveToPosition(LevelThreeMid, 5, 5, 2, 500);
//                drive.moveToPositionSlow(levelThreeDeposit, 5, 5, 2, 3000);
//                transfer.motors.get(0).setTargetPosition(-2280);
//                transfer.motors.get(0).setPower(80);
//                sleep(2000);
//                break;
//            case 1:
//                drive.moveToPositionSlow(levelOnepreGrab,10, 10,2, 1000);
//                drive.moveToPositionSlow(levelOneGrab,10, 10,2, 1500);
//                sleep(500);
//                cap.moveToStoringHeight();
//                drive.moveToPositionSlow(LevelOneMid, 5, 5, 2, 500);
//                drive.moveToPositionSlow(levelOneDeposit, 5, 5, 2, 2000);
//                drive.moveToPosition(levelOneDeposit2, 5, 5, 2, 3000);
//                transfer.motors.get(0).setTargetPosition(-1300);
//                transfer.motors.get(0).setPower(80);
//                break;
//        }
//        sleep(500);
//        output.servos.get(0).setPosition(0.3);
//        sleep(500);
//        output.servos.get(0).setPosition(0.7);
//        robot.odometry.reset();
//        transfer.motors.get(0).setTargetPosition(-600);
//        sleep(1000);
//        drive.moveToPositionSlow(SpecialBoyTurn, 5, 5, 2, 1500);
//        drive.odoUp();
//        sleep(500);
//        drive.motors.get(0).setPower(1);
//        drive.motors.get(1).setPower(1);
//        drive.motors.get(2).setPower(1);
//        drive.motors.get(3).setPower(1);
//        sleep(1000);
//        drive.motors.get(0).setPower(0);
//        drive.motors.get(1).setPower(0);
//        drive.motors.get(2).setPower(0);
//        drive.motors.get(3).setPower(0);
//    }
//}