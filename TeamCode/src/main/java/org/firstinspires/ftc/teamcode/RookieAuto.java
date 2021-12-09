package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "rookieAuto", group = "autonomous")
public class RookieAuto extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        //Start
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        //Scan barcode
        //Pick up team shipping element
        //Drive to shipping hub
        //Put crate on correct level
        //Drive to carousel
        //Move spinner to correct position
        //Spin carousel
        //Move to
        if (timer.time()<=20){
            //Warehouse
        } else{
            //Shipping unit
        }
    }
}
