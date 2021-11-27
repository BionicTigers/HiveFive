package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.*;

/*
Uses the gyro integrated into the expansion hub to return data on robot angles, velocity, and
linear acceleration on the x, y, and z axes
 */

@Autonomous(name="Gyro")
public class GyroIMU extends LinearOpMode{
    Orientation angles;
    Acceleration gravity;

    @Override public void runOpMode(){
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while(opModeIsActive()){
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
