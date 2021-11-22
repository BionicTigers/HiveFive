package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "transfer_test")
public class TransferTest extends LinearOpMode{
    public Transfer transfer;

    public void runOpMode(){
        transfer = new Transfer((DcMotorEx) hardwareMap.get(DcMotor.class, "transfer"));
        waitForStart();
        while(opModeIsActive()){
            transfer.update(gamepad1, gamepad2);
            transfer.write();
        }
    }
}
