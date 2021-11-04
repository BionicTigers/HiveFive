package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Sensor : MR Gyro", group = "Sensor")
public class GyroTest extends LinearOpMode{
    IntegratingGyroscope gyro; //Declares an integrated gyroscope with name "gyro"
    public void runOpMode(){
        gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");

        waitForStart();

        while(opModeIsActive()){
            //finds angular velocity of the gyroscope
            AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
            //finds the z angle of the gyroscope
            float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            //reports collected data
            telemetry.addLine()
                    .addData("dx", formatRate(rates.xRotationRate))
                    .addData("dy", formatRate(rates.yRotationRate))
                    .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
            telemetry.addData("angle", "%s deg", formatFloat(zAngle));
            telemetry.update();
    }
    }
    String formatRate(float rate){
        return String.format("%.3f", rate);
    }
    String formatFloat(float rate){
        return String.format("%.3f", rate);
    }
}
