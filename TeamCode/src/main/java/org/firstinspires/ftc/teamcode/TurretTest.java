package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Turret")
public class TurretTest extends LinearOpMode {

    public Turret turret;

    public void runOpMode(){
       turret = new Turret(hardwareMap.get(DcMotorEx.class, "turret"));

       waitForStart();

       turret.update(gamepad1, gamepad2);
       turret.write();
    }
}
