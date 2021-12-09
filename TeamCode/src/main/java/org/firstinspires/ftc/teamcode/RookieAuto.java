package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "rookieAuto", group = "autonomous")
public class RookieAuto extends LinearOpMode{
    public Robot robot;
    public Transfer transfer;
    public Output output;
    public Drivetrain drivetrain;
    public Spinner spinner;
    public Cap cap;
    //Position finding
    public Location location;
    public Odometry2 odometry;
    public enum Level {BOTTOM, MIDDLE, TOP};

    @Override
    public void runOpMode() throws InterruptedException {
        //Start
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        //Scan barcode
        //(Use camera method) Level lev = vuforia.readBarcode();
        //Pick up team shipping element
        cap.moveToIntakeHeight();
        cap.moveToStoringHeight();
        //Drive to shipping hub
        //location =...
        drivetrain.actuallyMoveToPosition(location, 0.1, 0.1, 0.1, 5); //Change info
        //Put crate on correct level
        //*
        switch (lev){
            case BOTTOM:
                output.
                break;
            case MIDDLE:
                output.
                break;
            case TOP:
                output.
                break;
        }
         //*/
        //Drive to carousel
        //Move spinner to correct position
        //Spin carousel
        //Move to:
        if (timer.seconds()<=20){
            //Warehouse
        } else{
            //Shipping unit
        }
    }
}
