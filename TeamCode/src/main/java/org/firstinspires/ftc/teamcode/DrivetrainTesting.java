package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/*
This class utilizes the drivetrain class in order to move the robot as well as utilizing the
gyro to return information on the angle, velocity, and acceleration of each axis of the drivetrain
*/

@TeleOp (name = "drivetrain_testing")
public class DrivetrainTesting extends LinearOpMode{
    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"}; //establishes motor names
    public Drivetrain drivetrain; //declares drivetrain
    public Robot robot; //declares robot

    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array


    public void runOpMode() {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "servo1"), hardwareMap.get(Servo.class, "servo2"), hardwareMap.get(Servo.class, "servo3"));
        robot.initMotors(motorNames);
        waitForStart();
        //establishes IMU parameters/variables
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //what runs constantly once play button is pressed
        while(opModeIsActive()) {
            drivetrain.update(gamepad1, gamepad2);
            drivetrain.write();
            //Returns values of angles, velocity, and acceleration on each axis
            telemetry.addData("orientation:", "Angle:x=%6.1f,y=%6.1f,z=%6.1f",
                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle,
                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle,
                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.addData("Velocity", "Vel: x=%6.1f,y=%6.1f,z=%6.1f",
                    imu.getVelocity().xVeloc,
                    imu.getVelocity().yVeloc,
                    imu.getVelocity().zVeloc);
            telemetry.addData("LinearAccel", "Dist: x=%6.1f,y=%6.1f,z=%6.1f",
                    imu.getLinearAcceleration().xAccel,
                    imu.getLinearAcceleration().yAccel,
                    imu.getLinearAcceleration().zAccel);
            telemetry.update();
        }
    }
}
