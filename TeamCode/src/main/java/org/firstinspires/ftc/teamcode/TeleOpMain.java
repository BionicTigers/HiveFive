package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
import java.util.regex.Pattern;

/*
This class calls the different mechanisms on the robot so that they can all be used based on the
controller input
*/

@TeleOp (name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode{
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"}; //establishes motor names
    public Drivetrain drivetrain; //declares drivetrain
    public Robot robot; //declares robot
    public Intake intake;
    public PositionalTransfer transfer;
//    public Transfer transfer;
    public Output output;
    public Cap cap;
    public Spinner spinner;
    public ColorSensor color;
//    public RevBlinkinLedDriver blinkinLedDriver;
//    RevBlinkinLedDriver.BlinkinPattern pattern;
//    RevBlinkinLedDriver.BlinkinPattern pattern2;
//    public ElapsedTime timer;
//    public int currentPattern = 1;
//    private RevBlinkinLedDriver.BlinkinPattern endgamePattern;

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array
//    private Deadline endgameFlashies = new Deadline(85, TimeUnit.SECONDS);
//    private Deadline endgameFlashiesOver = new Deadline(90,TimeUnit.SECONDS);



    public void runOpMode() {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry/*, hardwareMap.get(Servo.class, "SDriveL"), hardwareMap.get(Servo.class, "SDriveM"), hardwareMap.get(Servo.class, "SDriveR")*/);
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry, hardwareMap.get(DigitalChannel.class, "channel"));
//        spinner = new Spinner(hardwareMap.get(DcMotorEx.class,"spinner"));
//        transfer = new Transfer(hardwareMap.get(DcMotorEx.class, "transfer"));
        output = new Output(hardwareMap.get(Servo.class, "output"));
//        cap = new Cap(hardwareMap.get(Servo.class, "cap1"));
//        color = hardwareMap.get(ColorSensor.class, "color");
//        robot.initMotors(motorNames);
//        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
//        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;
//        pattern2 = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
//        endgamePattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE;
//        blinkinLedDriver.setPattern(pattern);


//        timer = new ElapsedTime();
//        Deadline endgameFlashiesOver = new Deadline(90,TimeUnit.SECONDS);
//        Deadline endgameFlashies = new Deadline(80,TimeUnit.SECONDS);
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while(!isStarted() && !isStopRequested()){

            telemetry.addData("IMU calibration status ", imu.isGyroCalibrated());
            telemetry.update();
        }
        //These lines set motors and servos to their default position once teleOp starts
        waitForStart();
//        spinner.servos.get(0).setPosition(0.1);
        output.servos.get(0).setPosition(.7);
//        drivetrain.odoUp();
//        cap.moveToStoringHeight();
        Mechanism[] mechanisms = {intake, transfer, output, /*spinner,*/ drivetrain, /*cap, robot.odometry*/};


//        endgameFlashies.reset();
//        endgameFlashiesOver.reset();
        //what runs constantly once play button is pressed
        while(opModeIsActive()) {
//            drivetrain.dashboardtelemetry.addData("red", color.red()/55.0);
//            drivetrain.dashboardtelemetry.addData("green", color.green()/55.0);
//            drivetrain.dashboardtelemetry.addData("blue", color.blue()/55.0);
//            drivetrain.dashboardtelemetry.addData("Spinner Position", spinner.motors.get(0).getCurrentPosition());
//            drivetrain.dashboardtelemetry.addData("Spinner Speed", spinner.motors.get(0).getPower());
//            drivetrain.dashboardtelemetry.addData("endgameFlashies", endgameFlashies.hasExpired());
//            drivetrain.dashboardtelemetry.addData("endgameFlashiesOver? :(", endgameFlashiesOver.hasExpired());
//            if(endgameFlashies.hasExpired() && !endgameFlashiesOver.hasExpired() && currentPattern != 3){
//                currentPattern = 3;
//                blinkinLedDriver.setPattern(endgamePattern);
//            }
//            else if(color.green()/55.0 > 10.5 && currentPattern !=2 && (!endgameFlashies.hasExpired() || endgameFlashiesOver.hasExpired()))//green when nothing = 8.4
//            {
//                blinkinLedDriver.setPattern(pattern2);
//                currentPattern = 2;
//            }
//            else if(currentPattern != 1 && !(color.red()/55.0 > 5.5) && (!endgameFlashies.hasExpired() || endgameFlashiesOver.hasExpired()))
//            {
//                blinkinLedDriver.setPattern(pattern);
//                currentPattern = 1;
//            }

            for (Mechanism mech : mechanisms) { //For each mechanism in the mechanism array
                mech.update(gamepad1, gamepad2); //Run their respective update methods
            }

            for (Mechanism mech : mechanisms) { //For each mechanism in the mechanism array
                mech.write(); //Run their respective write methods
            }

            drivetrain.dashboardtelemetry.addData("orientation:", "Angle:x=%6.1f,y=%6.1f,z=%6.1f",
                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle,
                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle,
                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            drivetrain.dashboardtelemetry.addData("Velocity", "Vel: x=%6.1f,y=%6.1f,z=%6.1f",
                    imu.getVelocity().xVeloc,
                    imu.getVelocity().yVeloc,
                    imu.getVelocity().zVeloc);
            drivetrain.dashboardtelemetry.addData("LinearAccel", "Dist: x=%6.1f,y=%6.1f,z=%6.1f",
                    imu.getLinearAcceleration().xAccel,
                    imu.getLinearAcceleration().yAccel,
                    imu.getLinearAcceleration().zAccel);


        }
    }
}
