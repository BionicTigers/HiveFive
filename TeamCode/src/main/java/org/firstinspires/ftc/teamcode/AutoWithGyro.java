package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous (name = "GyroAuto", group = "Autonomous")
public class AutoWithGyro extends LinearOpMode{
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
        public ColorSensor color;

        private Location position = new Location();
        private int[] wheels = {0, 1, 2, 3};
        private int mode;
        //still need location for deposit level 2 and 3, drop duck off at 3 btw
        private final Location postDropMove = new Location(-260,0,-350,0);
        private final Location preCarousel = new Location(-1057.30,0,-188.99,38.03);
        private final Location carousel = new Location(-1425.81,0,-230.70,37.49);
        private final Location preDuck = new Location(-1534.05,0,-598.11,90);
        private final Location duck = new Location(-915.22,0,-598.11,90);
        private final Location preTurn = new Location(-302.02, 0, -328.28, 356.67);
        private final Location finalTurn = new Location(-302.02, 0, -328.28, 446.67);

        private final Location levelOneDeposit = new Location (-259.58,0,-551.44,355.04);

        private final Location levelTwoDeposit = new Location (-250.83, 0,-434.72,356.14);

        private final Location levelThreeDeposit = new Location (-302.02, 0, -428.28, 356.67);

        public boolean hasFreight;
        @Override
        public void runOpMode() throws InterruptedException {
            robot = new Robot(this);
            drive = new Drivetrain(robot, wheels, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
            spinner = new Spinner(hardwareMap.get(DcMotorEx.class,"spinner"), hardwareMap.get(Servo.class, "carouselB"));
            transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry, hardwareMap.get(DigitalChannel.class, "channel"));
            cap = new Cap(hardwareMap.get(Servo.class, "capServo"));
            output = new Output(hardwareMap.get(Servo.class, "output"));
            vuforia = new Vuforia();
            time = new ElapsedTime();
            intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
            color = hardwareMap.get(ColorSensor.class, "color");
            Deadline stop = new Deadline(28, TimeUnit.SECONDS);
            Deadline warehouse = new Deadline(250, TimeUnit.MILLISECONDS);
            Deadline leftTurn = new Deadline(750, TimeUnit.MILLISECONDS);
            Deadline rightTurn = new Deadline(500, TimeUnit.MILLISECONDS);

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
                transfer.motors.get(0).setTargetPosition(600 * 223/312);
                transfer.motors.get(0).setPower(50);
                cap.servos.get(0).setPosition(0.05);
            }
            waitForStart();
            robot.odometry.reset();
            time.reset();
            transfer.motors.get(0).setTargetPosition(0);
            switch(mode) {
                case 2:
                    drive.moveToPositionSlow(levelTwoDeposit, 5, 5, 2, 2000);
                    transfer.motors.get(0).setTargetPosition(1275 * 223/312);
                    break;
                case 3:
                    drive.moveToPositionSlow(levelThreeDeposit, 5, 5, 2, 2000);
                    transfer.motors.get(0).setTargetPosition(2460 * 223/312);
                    break;
                default:
                    drive.moveToPositionSlow(levelOneDeposit, 5, 5, 2, 2000);
                    transfer.motors.get(0).setTargetPosition(837 * 223/312);
                    break;
            }
            transfer.motors.get(0).setPower(80);
            sleep(1000);
            output.servos.get(0).setPosition(1);
            sleep(500);
            output.servos.get(0).setPosition(0.7);
            sleep(1000);
            transfer.motors.get(0).setTargetPosition(600 * 223/312);
            drive.moveToPositionSlow(preTurn, 5, 5, 2, 2000);
            drive.moveToPositionSlow(finalTurn, 5, 5, 2, 2000);
            hasFreight = false;
            drive.odoUp();
            sleep(500);
            drive.motors.get(0).setPower(1);
            drive.motors.get(1).setPower(1);
            drive.motors.get(2).setPower(1);
            drive.motors.get(3).setPower(1);
            sleep(1000);
            drive.motors.get(0).setPower(0);
            drive.motors.get(1).setPower(0);
            drive.motors.get(2).setPower(0);
            drive.motors.get(3).setPower(0);
            while(!stop.hasExpired()){
                while(!hasFreight){
                    intake.motors.get(0).setPower(1);
                    while(!leftTurn.hasExpired()){
                    drive.motors.get(0).setPower(0.6);
                    drive.motors.get(2).setPower(0.6);
                    drive.motors.get(1).setPower(0.4);
                    drive.motors.get(3).setPower(0.4);
                    rightTurn.reset();
                        while(!rightTurn.hasExpired()){
                        drive.motors.get(0).setPower(0.4);
                        drive.motors.get(2).setPower(0.4);
                        drive.motors.get(1).setPower(0.6);
                        drive.motors.get(3).setPower(0.6);
                        leftTurn.reset();
                }}
                    if(color.green()/55.0 > 37.0){
                        hasFreight = true;
                    }
            }
                intake.motors.get(0).setPower(0);
        }}}
