package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "GyroTest")
public class GyroTest extends LinearOpMode{
    public Drivetrain drivetrain; //declares drivetrain
    public int[] motorNumbers = {0, 1, 2, 3};
    public Robot robot;
    public PositionalTransfer transfer;

    public void runOpMode() {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transfer"), telemetry, hardwareMap.get(DigitalChannel.class, "channel"));
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        drivetrain.odoUp();
        transfer.motors.get(0).setTargetPosition(600 * 223/312);
        transfer.motors.get(0).setPower(50);
        Deadline stop = new Deadline(250, TimeUnit.MILLISECONDS);
        while(!isStarted() && !isStopRequested()){
            telemetry.addData("IMU calibration status ", imu.isGyroCalibrated());
            telemetry.addData("orientation:", "Angle:x=%6.1f,y=%6.1f,z=%6.1f",
                    imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle,
                    imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle,
                    imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.update();
        }
        waitForStart();
        while(!isStopRequested()){
        if(gamepad1.a){
            while( imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle<3){
                drivetrain.motors.get(0).setPower(70);
                drivetrain.motors.get(1).setPower(70);
                drivetrain.motors.get(2).setPower(70);
                drivetrain.motors.get(3).setPower(70);
            }
            stop.reset();
            while(!stop.hasExpired()){
                idle();
            }
        }
        drivetrain.motors.get(0).setPower(0);
        drivetrain.motors.get(1).setPower(0);
        drivetrain.motors.get(2).setPower(0);
        drivetrain.motors.get(3).setPower(0);
    }
}
}
