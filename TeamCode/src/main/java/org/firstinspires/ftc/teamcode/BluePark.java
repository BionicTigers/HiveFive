package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "BluePark", group = "Autonomous")
public class BluePark extends LinearOpMode {
    private Robot robot;
    private Intake intake;
    private PositionalTransfer transfer;
    public Output output;
    private Drivetrain drive;
    private Spinner spinner;
    private double initialX;

    private int[] wheels = {0, 1, 2, 3};
    public void runOpMode(){
        robot = new Robot(this);
        drive = new Drivetrain(robot, wheels, telemetry);
        spinner = new Spinner(hardwareMap.get(DcMotorEx.class,"spinner"));
        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry, hardwareMap.get(DigitalChannel.class, "channel"));
        output = new Output(hardwareMap.get(Servo.class, "output"));
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));

        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        initialX = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        waitForStart();
        while (imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle < initialX+0.2 &&
                    !(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle > 45)&& opModeIsActive()) {
            drive.motors.get(0).setPower(1);
            drive.motors.get(1).setPower(1);
            drive.motors.get(2).setPower(1);
            drive.motors.get(3).setPower(1);
        }
        sleep(500);
        drive.motors.get(0).setPower(0);
        drive.motors.get(1).setPower(0);
        drive.motors.get(2).setPower(0);
        drive.motors.get(3).setPower(0);
    }
}
