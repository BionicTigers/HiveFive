//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.AutoStuff.Variables;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.internal.system.Deadline;
//
//import java.util.concurrent.TimeUnit;
//
//@Autonomous (name = "GyroAuto", group = "Autonomous")
//public class AutoWithGyro extends LinearOpMode {
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
//    private DistanceSensor frontDistance;
//    private DistanceSensor leftDistance;
//
//    private Location position = new Location();
//    private int[] wheels = {0, 1, 2, 3};
//    private int mode;
//    private double initialX;
//    //still need location for deposit level 2 and 3, drop duck off at 3 btw
//    private final Location postDropMove = new Location(-260, 0, -350, 0);
//    private final Location preCarousel = new Location(-1057.30, 0, -188.99, 38.03);
//    private final Location carousel = new Location(-1425.81, 0, -230.70, 37.49);
//    private final Location preDuck = new Location(-1534.05, 0, -598.11, 90);
//    private final Location duck = new Location(-915.22, 0, -598.11, 90);
//    private final Location preTurn = new Location(-302.02, 0, -100, 356.67);
//    private final Location finalTurn = new Location(-302.02, 0, -350, 86.67);
//    private final Location origin = new Location(0, 0, 0, 0);
//
//    private final Location levelOneDeposit = new Location(-259.58, 0, -551.44, 355.04);
//
//    private final Location levelTwoDeposit = new Location(-250.83, 0, -434.72, 356.14);
//
//    private final Location levelThreeDeposit = new Location(-302.02, 0, -428.28, 356.67);
//
//    private final Location exitScoring = new Location(500,0,850,270);
//
//    public boolean hasFreight;
//    public boolean inWarehouse;
//
//    public Location outOfWarehouseReset;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        //declaring all the hardware devices and subclasses
//        robot = new Robot(this);
//        drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
//        spinner = new Spinner(hardwareMap.get(DcMotorEx.class, "spinner"), hardwareMap.get(Servo.class, "carouselB"));
//        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry, hardwareMap.get(DigitalChannel.class, "channel"));
//        cap = new Cap(hardwareMap.get(Servo.class, "capServo"));
//        output = new Output(hardwareMap.get(Servo.class, "output"));
//        vuforia = new Vuforia();
//        time = new ElapsedTime();
//        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
//        color = hardwareMap.get(ColorSensor.class, "color");
//        //timers for things in auto
//        Deadline stop = new Deadline(28, TimeUnit.SECONDS);
//        Deadline warehouse = new Deadline(250, TimeUnit.MILLISECONDS);
//        Deadline leftTurn = new Deadline(750, TimeUnit.MILLISECONDS);
//        Deadline rightTurn = new Deadline(1, TimeUnit.SECONDS);
//        //distance sensors
//        frontDistance = new DistanceSensor(hardwareMap.get(AnalogInput.class, "frontDistance"), telemetry, "Front");
//        leftDistance = new DistanceSensor(hardwareMap.get(AnalogInput.class, "leftDistance"), telemetry, "Left");
//        //imu stuff
//        BNO055IMU imu;
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//        //odo setup
//        robot.odometry.reset();
//        drive.odoDown();
//        //initializing the camera
//        OpenCvCamera webcam;
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.openCameraDevice();
//        webcam.setPipeline(new Vuforia());
//        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//        vuforia = new Vuforia(webcam);
//        spinner.servos.get(0).setPosition(0.48);
//
//        while (!isStarted() && !isStopRequested()) {
//            robot.odometry.updatePosition();
//            drive.telemetry.addData("Odometry", robot.odometry.getPosition().getLocation(0) + ", " + robot.odometry.getPosition().getLocation(2) + ", " + robot.odometry.getPosition().getLocation(3));
//            mode = vuforia.getMode();
//            telemetry.addData("Mode", mode);
//            telemetry.addData("Area", vuforia.getArea());
//
//            transfer.motors.get(0).setTargetPosition(600 * 223 / 312);
//            transfer.motors.get(0).setPower(50);
//            cap.servos.get(0).setPosition(0.05);
//            //to make sure we have the gyro calibrated before starting auto
//            telemetry.addData("IMU calibration status ", imu.isGyroCalibrated());
//            telemetry.addData("orientation:", "Angle:x=%6.1f,y=%6.1f,z=%6.1f",
//                    imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle,
//                    imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle,
//                    imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
//            telemetry.update();
//            initialX = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
//        }
//        waitForStart();
//        stop.reset();
//        robot.odometry.reset();
//        time.reset();
//        transfer.motors.get(0).setTargetPosition(0);
//        switch (mode) {
//            case 2:
//                drive.moveToPositionSlow(levelTwoDeposit, 5, 5, 2, 2000);
//                transfer.motors.get(0).setTargetPosition(1275 * 223 / 312);
//                break;
//            case 3:
//                drive.moveToPositionSlow(levelThreeDeposit, 5, 5, 2, 2000);
//                transfer.motors.get(0).setTargetPosition(2460 * 223 / 312);
//                break;
//            default:
//                drive.moveToPositionSlow(levelOneDeposit, 5, 5, 2, 2000);
//                transfer.motors.get(0).setTargetPosition(837 * 223 / 312);
//                break;
//        }
//        transfer.motors.get(0).setPower(80);
//        sleep(1000);
//        output.servos.get(0).setPosition(1);
//        sleep(500);
//        output.servos.get(0).setPosition(0.7);
//        sleep(1000);
//        transfer.motors.get(0).setTargetPosition(600 * 223 / 312);
//
//        while (!stop.hasExpired()&& opModeIsActive()) {
//            drive.moveToPositionSlow(preTurn, 5, 5, 2, 500);
//            drive.moveToPositionSlow(finalTurn, 5, 5, 2, 2000);
//            hasFreight = false;
//            drive.odoUp();
//            telemetry.addLine("About to drive");
//            telemetry.update();
//
//            //enters warehouse
//            while (imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle < initialX+0.2 &&
//                    !(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle > 45)&& opModeIsActive()) {
//                telemetry.addLine("In IMU loop");
//                telemetry.addData("x position: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
//                telemetry.update();
//                drive.motors.get(0).setPower(1);
//                drive.motors.get(1).setPower(1);
//                drive.motors.get(2).setPower(1);
//                drive.motors.get(3).setPower(1);
//                inWarehouse = true;
//            }
//            sleep(400);
//            drive.motors.get(0).setPower(0);
//            drive.motors.get(2).setPower(0);
//            drive.motors.get(1).setPower(0);
//            drive.motors.get(3).setPower(0);
//            drive.odoDown();
//            robot.odometry.reset();
//            rightTurn.reset();
//            while (!hasFreight&& opModeIsActive()) {
//
//
//                //sets warehouse flag to true, runs intake while swerving back and forth and constantly checking for freight
//                inWarehouse = true;
//                sleep(100);
//                intake.motors.get(0).setPower(1);
//                transfer.motors.get(0).setTargetPosition(0);
//                sleep(200);
//                while (!rightTurn.hasExpired() && !hasFreight && !stop.hasExpired()&& opModeIsActive()) {
//                    drive.motors.get(0).setPower(0.4);
//                    drive.motors.get(2).setPower(0);
//                    drive.motors.get(1).setPower(0);
//                    drive.motors.get(3).setPower(0.4);
//                    if (color.green() / 55.0 > 38) {
//                        hasFreight = true;
//                    }
//                    robot.odometry.updatePosition();
//                    drive.telemetry.addData("Odometry x:", robot.odometry.getPosition().getLocation(0) + " \n z: " + robot.odometry.getPosition().getLocation(2) + "\n rot:" + robot.odometry.getPosition().getLocation(3));
//                    drive.telemetry.update();
//                }
//
////                leftTurn.reset();
////                while (!leftTurn.hasExpired() && !hasFreight && !stop.hasExpired()) {
////                    drive.motors.get(0).setPower(0.4);
////                    drive.motors.get(2).setPower(0);
////                    drive.motors.get(1).setPower(0);
////                    drive.motors.get(3).setPower(0.4);
////                    if (color.green() / 55.0 > 37.0) {
////                        hasFreight = true;
////                    }
////
////                }
//
//
//                drive.motors.get(0).setPower(0);
//                drive.motors.get(2).setPower(0);
//                drive.motors.get(1).setPower(0);
//                drive.motors.get(3).setPower(0);
//                transfer.motors.get(0).setTargetPosition(600 * 223 / 312);
//
//                sleep(100);
//                rightTurn.reset();
//                if (color.green() / 55.0 > 38) {
//                    hasFreight = true;
//                }
//            }
//            intake.motors.get(0).setPower(-1);
//            drive.moveToPositionSlow(origin, 5, 5, 2, 2000);
//            transfer.motors.get(0).setTargetPosition(600*223/312);
//            drive.odoUp();
//            initialX =  imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
//            // leave warehouse
//            while (imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle < initialX+0.2 && opModeIsActive()) {
//                drive.dashboardtelemetry.addLine("Second IMU loop");
//                drive.dashboardtelemetry.addData("x position: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
//                drive.dashboardtelemetry.update();
//                drive.motors.get(0).setPower(-1);
//                drive.motors.get(1).setPower(-1);
//                drive.motors.get(2).setPower(-1);
//                drive.motors.get(3).setPower(-1);
//            }
//            sleep(500);
//            drive.motors.get(0).setPower(0);
//            drive.motors.get(2).setPower(0);
//            drive.motors.get(1).setPower(0);
//            drive.motors.get(3).setPower(0);
//
//             inWarehouse = false;
//           for( int i =0; i < 3; i++ ){
//                frontDistance.getDistance();
//                leftDistance.getDistance();
//                sleep(100);
//            }
//           telemetry.addData("I reset x with", frontDistance.getAverage());
//           telemetry.addData("I reset z with", leftDistance.getAverage());
//           telemetry.addData("I reset rot with", Math.abs(imu.getAngularOrientation(AxesReference.EXTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).secondAngle));
////            outOfWarehouseReset = new Location(frontDistance.getAverage(), 0, leftDistance.getAverage(), Math.abs(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle));
////            robot.odometry.reset(outOfWarehouseReset);
////            drive.moveToPositionSlow(exitScoring,5,5,2,2000);
//            sleep(20000);
//            break;
//        }
//
//    }
//}
