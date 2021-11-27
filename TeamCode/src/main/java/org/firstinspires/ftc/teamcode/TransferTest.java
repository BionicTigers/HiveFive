package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/*
Calls the transfer mechanism class to allow it to actually move
*/

@TeleOp (name = "transfer_test")
public class TransferTest extends LinearOpMode{
    public Transfer transfer; //declares transfer

    public void runOpMode(){
        //establishes the transfer motor
        transfer = new Transfer((DcMotorEx) hardwareMap.get(DcMotor.class, "transfer"));
        waitForStart();
        //what actually happens when the code is running
        while(opModeIsActive()){
            transfer.update(gamepad1, gamepad2); //updates values of variables
            transfer.write(); //sets power for the motor
        }
    }
}
