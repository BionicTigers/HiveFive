package org.firstinspires.ftc.teamcode.WorldsAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;
import org.firstinspires.ftc.teamcode.Cap;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Location;
import org.firstinspires.ftc.teamcode.Output;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Spinner;
import org.firstinspires.ftc.teamcode.SuperiorVision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

//OG auto
@Autonomous(name="Supvis")
public class Supvistest extends LinearOpMode {
    private Robot robot;
    private Variables variables;
    private SuperiorVision superiorvision;
    private ElapsedTime time;

    private Location position = new Location();
    private int[] wheels = {0, 1, 2, 3};
    private int mode;
    public boolean hasFreight;
    //still need location for deposit level 2 and 3, drop duck off at 3 btw
    private final Location postDropMove = new Location(-260, 0, -350, 0);
    private final Location preCarousel = new Location(-1057.30, 0, -188.99, 38.03);
    private final Location carousel = new Location(-1425.81, 0, -200.70, 39);
    private final Location preDuck = new Location(-1534.05, 0, -612, 93);
    private final Location duck = new Location(-885.22, 0, -622, 93);
    private final Location preTurn = new Location(-302.02, 0, -328.28, 356.67);
    private final Location finalTurn = new Location(-302.02, 0, -328.28, 86.67);
    private final Location origin = new Location(0, 0, 125, 0);
    private final Location preHubDuck = new Location(-800, 0, -400, 360);

    private final Location levelOneDeposit = new Location(-259.58, 0, -531.44, 0);

    private final Location levelTwoDeposit = new Location(-308.83, 0, -530, 356.14);

    private final Location levelThreeDeposit = new Location(-302.02, 0, -428.28, 356.67);

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(this);
        superiorvision = new SuperiorVision();
        time = new ElapsedTime();
        Deadline stop = new Deadline(28, TimeUnit.SECONDS);
        Deadline rightTurn = new Deadline(1, TimeUnit.SECONDS);


        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new SuperiorVision.SkystoneDeterminationPipeline());
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        superiorvision = new SuperiorVision();
        waitForStart();

    }
}