package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Location;

//@Config
public class Variables {
    public static double linServo = .5;

    public static float mult = .3f;

    public static double intakePower = .8; //Change!!!
    // ?
    public static double kfP=0.003;
    public static double kfI=0.000005;
    public static double kfD = 0;
    public static double ksP=0.0039;
    public static double ksI=0.000005;
    public static double ksD = 0;
    public static double krp =.045;
    public static double krI=0.000035;
    //Shooting position
    public static Location shootPos = new Location(1400.8153f,0f,-674f,352.1f);
    public static double krD = 0.00;

    public static double setPointRotation = 0;

    public static double fp;
    public static double fi;
    public static double fd;
    public static double sp;
    public static double si;
    public static double sd;
    public static double rp;
    public static double ri;
    public static double rd;
    public static double midOdo = 202.0;
    public static double wfP;
    public static double wsP;
    public static double wrP;
    public static double wfI;
    public static double wsI;
    public static double wrI;
    public static double wfD;
    public static double wsD;
    public static double wrD;
    //Tolerances
    public static double xTolernce = 5;
    public static double yTolerance = 5;
    public static double rotTolerance = 0.25;
    public static Location transitionLocation = new Location();

    //2020-2021 code
    //Target variables
    public static double targetRot = 0;
    public static double targetz = 400;
    public static double targetx = 800;
    //Shooting variables
    public static Object ShooterVars;
    public static double shooterP = 200;
    public static double shooterP2 = 300;
    public static double shooterI = 0.9;
    public static double shooterI2 = 8;
    public static double shooterD = 5;
    public static double shooterD2 = 0;
    public static double shooterF = 9.4;
    public static double shooterF2 = 9.4;
    //Wobble goal variables
    public static double goalSpeed = 1540;
    public static double wobbleUpArmEncoderPos = -550;
    public static double wobbleDownArmEncoderPos = -1400;
    public static int wobbleGoalTrim;
    //Blocker?
    public static double blocker1Up=0;
    public static double blocker2Up=1;
    public static double blocker1Down=.42;
    public static double blocker2Down=.65;
    public static float trim=0;
}