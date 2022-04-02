package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret")
public class TurretTest extends LinearOpMode {

    public Turret turret;
    public PositionalLift lift;
    public void runOpMode(){
       turret = new Turret(hardwareMap.get(DcMotorEx.class, "turret"));
       lift = new PositionalLift(hardwareMap.get(DcMotorEx.class, "lift"), telemetry);
        waitForStart();
        while(!isStopRequested()) {
            turret.update(gamepad1, gamepad2);
            turret.write();
            lift.update(gamepad1, gamepad2);
            lift.write();
        }
    }
}
