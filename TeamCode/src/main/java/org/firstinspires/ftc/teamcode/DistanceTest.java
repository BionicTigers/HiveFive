package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;


@TeleOp(name = "distance")
public class DistanceTest extends LinearOpMode{
    AnalogInput frontdistance;
    AnalogInput leftdistance;
    double a = 0;
    double b = 0;
    double c = 0;
    double average;
    @Override
    public void runOpMode(){
        frontdistance = hardwareMap.get(AnalogInput.class, "frontDistance");
        leftdistance = hardwareMap.get(AnalogInput.class, "leftDistance");
        c=frontdistance.getVoltage()*2249.172474-212.0578341;
        waitForStart();
        while (opModeIsActive()) {
            if (c!=frontdistance.getVoltage()*2249.172474-212.0578341) {
                a = b;
                b = c;
                c = frontdistance.getVoltage() * 2249.172474 - 212.0578341;
                average = (a + b + c) / 3.0;
            }
            telemetry.addData("Distance (mm):", average);
            telemetry.addData("Voltage  :", frontdistance.getVoltage());
            telemetry.addData("A", a);
            telemetry.addData("B", b);
            telemetry.addData("C", c);
            telemetry.update();
        }
    }
}
