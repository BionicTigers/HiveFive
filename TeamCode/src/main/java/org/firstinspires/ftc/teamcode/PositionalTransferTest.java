package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "pos_transfer")
public class PositionalTransferTest extends LinearOpMode{
    public PositionalTransfer transfer;

    public void runOpMode(){
        transfer = new PositionalTransfer(hardwareMap.get(DcMotorEx.class, "transferMotor"));
        waitForStart();

        while(opModeIsActive()){
            transfer.update(gamepad1, gamepad2);
            transfer.write();
        }
    }
}
