package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/*
This class utilizes the drivetrain class in order to move the robot as well as utilizing the
gyro to return information on the angle, velocity, and acceleration of each axis of the drivetrain
*/

@TeleOp (name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode{
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"}; //establishes motor names
    public Drivetrain drivetrain; //declares drivetrain
    public Robot robot; //declares robot
    public Intake intake;
    public PositionalTransfer transfer;
    public Output output;
   // public Cap cap;
    public Spinner spinner;

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array


    public void runOpMode() {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"), hardwareMap.get(Servo.class, "intakeLeft"), hardwareMap.get(Servo.class, "intakeRight"), hardwareMap.get(Servo.class, "blocker"));
        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry);
        output = new Output(hardwareMap.get(Servo.class, "output"));
        spinner = new Spinner(hardwareMap.get(CRServo.class, "spinner"), hardwareMap.get(Servo.class, "carouselB"));
//        cap = new Cap(hardwareMap.get(Servo.class, "capServo"));
        robot.initMotors(motorNames);
        //These lines set motors and servos to their default position once teleOp starts
        waitForStart();
        //establishes IMU parameters/variables
//        BNO055IMU imu;
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);

        intake.servos.get(0).setPosition(0.83);
        intake.servos.get(1).setPosition(0.7);
        intake.servos.get(2).setPosition(0.1);
        output.servos.get(0).setPosition(1);
        drivetrain.odoUp();

        Mechanism[] mechanisms = {intake, transfer, output, spinner, drivetrain, robot.odometry};

        //what runs constantly once play button is pressed
        while(opModeIsActive()) {

            for (Mechanism mech : mechanisms) { //For each mechanism in the mechanism array
                mech.update(gamepad1, gamepad2); //Run their respective update methods
            }

            for (Mechanism mech : mechanisms) { //For each mechanism in the mechanism array
                mech.write(); //Run their respective write methods
            }

            //Returns values of angles, velocity, and acceleration on each axis
//            telemetry.addLine("IMU Data");
//            telemetry.addData("orientation:", "Angle:x=%6.1f,y=%6.1f,z=%6.1f",
//                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle,
//                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle,
//                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
//            telemetry.addData("Velocity", "Vel: x=%6.1f,y=%6.1f,z=%6.1f",
//                    imu.getVelocity().xVeloc,
//                    imu.getVelocity().yVeloc,
//                    imu.getVelocity().zVeloc);
//            telemetry.addData("LinearAccel", "Dist: x=%6.1f,y=%6.1f,z=%6.1f",
//                    imu.getLinearAcceleration().xAccel,
//                    imu.getLinearAcceleration().yAccel,
//                    imu.getLinearAcceleration().zAccel);
            telemetry.update();
        }
    }
}
