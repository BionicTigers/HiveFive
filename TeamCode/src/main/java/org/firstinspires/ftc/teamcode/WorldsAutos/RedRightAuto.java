//package org.firstinspires.ftc.teamcode.WorldsAutos;
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
//import org.firstinspires.ftc.teamcode.EvilVision;
//import org.firstinspires.ftc.teamcode.Intake;
//import org.firstinspires.ftc.teamcode.Location;
//import org.firstinspires.ftc.teamcode.Output;
//import org.firstinspires.ftc.teamcode.PositionalTransfer;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.Spinner;
//import org.firstinspires.ftc.teamcode.Turret;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.concurrent.TimeUnit;
//
//@Autonomous(name="RightRed")
//public class RedRightAuto extends LinearOpMode {
//    //Objects
//    private Robot robot;
//    private Intake intake;
//    private PositionalTransfer transfer;
//    private Cap cap;
//    public Output output;
//    private Drivetrain drive;
//    private Variables variables;
//    private EvilVision evilvision;
//    private Spinner spinner;
//    private ElapsedTime time;
//    public ColorSensor color;
//    private Turret turret;
//
//    //Extender positions
//    private int extenderCarousel = 30; //Change
//    private int extenderStorage = 0;
//
//    //Elevator positions
//    private int elevatorCarousel = 30; //Change
//    private int elevatorBottom = 30; //Change
//    private int elevatorMiddle = 30; //Change
//    private int elevatorTop = 30; //Change
//    private int elevatorStorage = 0;
//
//    //Rotate positions
//    private int rotateStorage = 0;
//    private int rotateCarousel = 0;
//        //add specific rotate angles
//
//    //Changing data
//    private Location position = new Location();
//    private int[] wheels = {0, 1, 2, 3};
//    private int mode;
//
//    //Locations
//    private final Location carousel = new Location(0, 0, 0, 0); //Add numbers
//    private final Location warehouse = new Location(0, 0, 0, 0); //Add numbers
//    private final Location park = new Location(0, 0, 0, 0); //Add numbers
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        //
//        robot = new Robot(this);
//        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDriveL"), hardwareMap.get(Servo.class, "SDriveM"), hardwareMap.get(Servo.class, "SDriveR"));
//        spinner = new Spinner(hardwareMap.get(DcMotorEx.class,"spinner"));
//        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry, hardwareMap.get(DigitalChannel.class, "channel"), hardwareMap.get(Servo.class, "STransfer1"), hardwareMap.get(Servo.class, "STransfer2"));
//        cap = new Cap(hardwareMap.get(CRServo.class, "cap1"), hardwareMap.get(Servo.class, "cap2"));
//        output = new Output(hardwareMap.get(Servo.class, "output"));
//        evilvision = new EvilVision();
//        time = new ElapsedTime();
//        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
//        color = hardwareMap.get(ColorSensor.class, "color");
//        OpenCvCamera webcam;
//
//        robot.odometry.reset();
//        drive.odoDown();
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.openCameraDevice();
//        webcam.setPipeline(new EvilVision());
//        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//        evilvision = new EvilVision(webcam);
//
//        while (!isStarted() && !isStopRequested()) {
//            robot.odometry.updatePosition();
//            drive.telemetry.addData("Odometry", robot.odometry.getPosition().getLocation(0) + ", " + robot.odometry.getPosition().getLocation(2) + ", " + robot.odometry.getPosition().getLocation(3));
//            mode = evilvision.getMode();
//            telemetry.addData("Mode", mode);
//            telemetry.addData("Area", evilvision.getArea());
//            telemetry.update();
//            if (gamepad1.a) {
//                robot.odometry.reset();
//            }
//            transfer.motors.get(0).setTargetPosition(175);
//            transfer.motors.get(0).setPower(50);
//
//            spinner.motors.get(0).setTargetPosition(0);
//            spinner.motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        //On start
//        waitForStart();
//        robot.odometry.reset();
//        time.reset();
//        transfer.motors.get(0).setTargetPosition(0);
//        //Score preload on carousel
//        //Moves slide
//        turret.moveExtender(extenderCarousel);
//        turret.rotateTurret(rotateCarousel);
//        sleep(1000);
//        //Switch to determine which elevator level to deposit preload on
//        switch(mode) {
//            //Move elevator to second level
//            case 2:
//                turret.moveElevator(elevatorMiddle);
//                break;
//            //Move elevator to third level
//            case 3:
//                turret.moveElevator(elevatorTop);
//                break;
//            //Move elevator to first level
//            default:
//                turret.moveElevator(elevatorBottom);
//                break;
//        }
//        //Deposits freight
//        intake.deposit();
//        //Resets turret position
//        turret.moveElevator(elevatorStorage);
//        turret.moveExtender(extenderStorage);
//        turret.rotateTurret(rotateStorage);
//        sleep(1000);
//
//        //Cycle
//        Deadline cycleTime = new Deadline(25, TimeUnit.SECONDS);
//        while(!cycleTime.hasExpired()){
//            //Moves to warehouse
//            drive.moveToPosition(warehouse, 5, 5, 5);
//            sleep(2000);
//            //Intakes freight
//            intake.intake();
//            sleep(5);
//            //Leaves warehouse
//            drive.moveToPosition(carousel, 5, 5, 5);
//            //Moves turret to scoring positions
//            turret.rotateTurret(rotateCarousel);
//            turret.moveElevator(elevatorTop);
//            turret.moveExtender(extenderCarousel);
//            sleep(1000);
//            //Deposits
//            intake.deposit();
//            sleep(5);
//            //Moves turret to storage positions
//            turret.rotateTurret(rotateStorage);
//            turret.moveElevator(elevatorStorage);
//            turret.moveExtender(extenderStorage);
//            sleep(1000);
//        }
//        drive.moveToPosition(park, 5, 5, 5);
//        drive.odoUp();
//    }
//}
