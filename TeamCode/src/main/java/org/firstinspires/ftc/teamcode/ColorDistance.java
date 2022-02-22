package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "ColorDistance")
public class ColorDistance extends LinearOpMode{
    ColorSensor color;
    DistanceSensor distance;

    public void runOpMode() {
        color = hardwareMap.get(ColorSensor.class, "color");
        distance = hardwareMap.get(DistanceSensor.class, "color");
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        waitForStart();

        while(opModeIsActive()){
            Color.RGBToHSV((int) (color.red() * SCALE_FACTOR),
                    (int) (color.green() * SCALE_FACTOR),
                    (int) (color.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", distance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", color.alpha());
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Hue", hsvValues[0]);

            relativeLayout.post(new Runnable(){
                public void run(){
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });
            telemetry.update();
        }
        relativeLayout.post(new Runnable(){
            public void run(){
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}
