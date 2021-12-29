package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

@TeleOp (name = "single wheel")
public class OdoWheelTest extends LinearOpMode {

    public Odometry odometry;
    public Telemetry telemetry;
    public Drivetrain drivetrain;
    public RevBulkData bulkdata;
    public Robot robot; //declares robot
    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array
    public ExpansionHubEx expansionHub;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "SDrive1"), hardwareMap.get(Servo.class, "SDrive2"), hardwareMap.get(Servo.class, "SDrive3"));
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        bulkdata = expansionHub.getBulkInputData();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Odo wheel position", bulkdata.getMotorCurrentPosition(0));
            telemetry.update();
        }
    }
}