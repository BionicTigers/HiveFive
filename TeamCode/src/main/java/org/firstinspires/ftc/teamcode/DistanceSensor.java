package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DistanceSensor extends Mechanism{
    AnalogInput distanceI;
    double a = 0;
    double b = 0;
    double c = 0;
    double average;
    public Telemetry telemetry;
    String names;

    public DistanceSensor(AnalogInput distance, Telemetry T, String name){
        telemetry = T;
        distanceI = distance;
        names = name;
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {

        if (c != distanceI.getVoltage() * 1764.642713 - 1329.143288) {
            a = b;
            b = c;
            c = distanceI.getVoltage() * 1764.642713 - 1329.143288;
            average = ((a + b + c) / 3.0);
        }
    }


    @Override
    public void write() {
        telemetry.addData(names + " Distance (mm):", average);
        telemetry.addData("Voltage  :", distanceI.getVoltage());
        telemetry.addData("A", a);
        telemetry.addData("B", b);
        telemetry.addData("C", c);
    }
    public void getDistance(){
        if (c != distanceI.getVoltage() * 1764.642713 - 1329.143288) {
            a = b;
            b = c;
            c = distanceI.getVoltage() * 1764.642713 - 1329.143288;
            average = ((a + b + c) / 3.0);
        }
    }
    public double getAverage()
    {
        return average;
    }
}
