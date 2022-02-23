package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;


@TeleOp(name = "distance")
public class DistanceTest extends LinearOpMode{
    AnalogInput distance;

    @Override
    public void runOpMode(){
        distance = hardwareMap.get(AnalogInput.class, "distance");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Distance (mm):", (distance.getVoltage()*2249.172474-272.0578341));
            telemetry.addData("Distance (mm):", distance.getVoltage());
            telemetry.update();
        }
    }
}
