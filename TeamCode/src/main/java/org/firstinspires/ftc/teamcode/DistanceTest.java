package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
public class DistanceTest extends LinearOpMode{
    DistanceSensor distance;

    @Override
    public void runOpMode(){
        distance = hardwareMap.get(DistanceSensor.class, "Distance");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
