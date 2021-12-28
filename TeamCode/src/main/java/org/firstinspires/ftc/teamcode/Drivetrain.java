package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Mechanism;
import org.firstinspires.ftc.teamcode.Location;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

public class Drivetrain extends Mechanism {

    //Fields
    public Robot robot;
    public Telemetry dashboardTelemetry;
    public double[] motorPowers; //powers for the motors for the wheels
    //private ArrayList<DcMotorEx> dtMotors = new ArrayList<DcMotorEx>();
    private int[] dtMotorNumbers; //the numbers of the indices where the drivetrain motors will be stored
    private Location forth = new Location (0, 0, 1000, 0);
    private Location backth = new Location (0, 0, -1000, 0);
    private Location lefth = new Location (-500, 0, 0, 0);
    private Location righth = new Location (500, 0, 0, 0);
    private Location clockth = new Location (0, 0, 0, 90);
    private Location counterth = new Location (0, 0, 0, 270);
    private Location centerth = new Location (0, 0, 0, 0);

    //All the PID/Movement stuffs
    private Odometry odo;

    //Spin PID variables
    public double spinError;
    public double previousSpinError=20;
    public double timeWhenLeave;

    // array that contains the different Integral Values
    public double[] integralValues=new double[4];

    //Telemetry for the drivetrain
    public Telemetry telemetree;

    public Location error = new Location();
    public boolean testing = false;

    //Power Shot Variables

    private boolean doPowerShots = false; //Declares whether the code should try to do power shots
    private boolean yCurrentlyPressed = false; //Declares whether Y is currently being held
    public int powerShotNumber = 1; /* Declares what phase of power shots the code should run
    1 == power shot 1
    2 == power shot 2
    3 == power shot 3
    4 == stop power shots */
    private long powerServoStartTime = 0; //Declares when the servo started spinning
    private boolean acceptableSpeed = true; //Declares whether the speed has been acceptable at any point
    private boolean restartPowerTimer = true; //Declares whether the timer for the servo should reset
    //Sets positions for each power shot, X, Z, and rotation
    private final Location POWER_SHOT1 = new Location(-1281.09f,0, 1460.34f, 355);
    private final Location POWER_SHOT2 = new Location(-1501.86f, 0, 1460.34f, 355);
    private final Location POWER_SHOT3 = new Location(-1723.08f, 0, 1460.34f, 355);



    private double lastForwardError;
    private double lastSidewaysError;
    private double lastRotationError;

    private PID dtPIDx;
    private PID dtPIDz;
    private PID dtPIDr;
    private PID[] dtPIDs = {dtPIDx, dtPIDz, dtPIDr};
    public boolean autoIsDone = false;
    private Location powerShotError;
    private double magnitude;
    private double robotheading;
    private Deadline powerShootingDeadline;
    private boolean firstTimeForShot = true;
    private boolean isHomed =false;

    //Constructors for Drivetrain class, creates the amount of motors passed in and initializes robot value to whatever is passed in.

    // Assigns the motors of the drivetrain to the indices in the motor ArrayList of the Robot class whichever numbers are in the int[] motorNumbers
    public Drivetrain(Robot bot, int[] motorNumbers, Telemetry telem){
        DcMotorEx motorPlaceholder;
        robot = bot;
        dtMotorNumbers = motorNumbers;
        for(int motNum: motorNumbers){
            motorPlaceholder = robot.motors.get(motNum);
            motorPlaceholder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.add(motorPlaceholder);
        }
        motorPowers = new double[]{0, 0, 0, 0};
        odo = bot.odometry;
        telemetree = telem;
//        dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = dashboard.getTelemetry();
        powerShootingDeadline = new Deadline(2000, TimeUnit.MILLISECONDS);
    }

    public Drivetrain(Robot bot, int[] motorNumbers, Telemetry telem, PID pidx, PID pidz, PID pidr){
        DcMotorEx motorPlaceholder;
        robot = bot;
        dtMotorNumbers = motorNumbers;
        for(int motNum: motorNumbers){
            motorPlaceholder = robot.motors.get(motNum);
            motorPlaceholder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            motors.add(motorPlaceholder);
        }
        motorPowers = new double[]{0, 0, 0, 0};
        odo = robot.odometry;
        telemetree = telem;
//        dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = dashboard.getTelemetry();
        dtPIDx = new PID(pidx);
        dtPIDz = new PID(pidz);
        dtPIDr = new PID(pidr);
    }

    public Drivetrain(Robot bot, int[] motorNumbers){
        DcMotorEx motorPlaceholder;
        robot = bot;
        dtMotorNumbers = motorNumbers;
        for(int motNum: motorNumbers){
            motorPlaceholder = robot.motors.get(motNum);
            motorPlaceholder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.add(motorPlaceholder);
        }
        motorPowers = new double[]{0, 0, 0, 0};
        odo = robot.odometry;
//        dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = dashboard.getTelemetry();
    }

//    public Drivetrain(Robot bot, int[] motorNumbers, Telemetry telem, Shooter2 shoot) {
//        DcMotorEx motorPlaceholder;
//        robot = bot;
//        dtMotorNumbers = motorNumbers;
//        for(int motNum: motorNumbers){
//            motorPlaceholder = robot.motors.get(motNum);
//            motorPlaceholder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            motors.add(motorPlaceholder);
//        }
//        motorPowers = new double[]{0, 0, 0, 0};
//        odo = robot.odometry;
//        telemetree = telem;
//        dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = dashboard.getTelemetry();
//        shooter = shoot;
//        powerShootingDeadline = new Deadline(2000, TimeUnit.MILLISECONDS);
//
//
//    }
    public Drivetrain(@NonNull org.firstinspires.ftc.teamcode.Robot bot, @NonNull int[] motorNumbers, Telemetry T, Servo SDrive1, Servo SDrive2, Servo SDrive3) {
        DcMotorEx motorPlaceholder;
        robot = bot;
        telemetree = T;
        //odo = bot.odometry;

        getServos().add(SDrive1);
        getServos().add(SDrive2);
        getServos().add(SDrive3);

        for (int motNum : motorNumbers) {
            motorPlaceholder = robot.motors.get(motNum);
            motorPlaceholder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.add(motorPlaceholder);
        }
        motorPowers = new double[]{0, 0, 0, 0};
    }

    //This method determines the values of the array list motorPowers with the four motor powers for the drivetrain with the mecanum calculations!
    //Because mecanum wheels have diagonal rollers, we must do some special calculations to determine which way is forward and what powers to use to strafe.
    public void determineMotorPowers(Gamepad driverPad) {
        //P is the power
        //robotAngle is the angle to which you want to go
        //rightX is the value of the x-axis from the right joystick
        // the vals are added to their respective directions to achieve slow percision conrtrol
//-----------------------------------------------------------------------------------------------------
        //im using ternary operators to flex here its a fancy if statement
        // basically if the statement before the question mark is true the whole thing will be substituted with the first thing after the ?
        // ex if dpadUp is true the whole expression will evaluate to .2
        // if the condition is false it will do the second thing in this example if dpad up is not pressed it will evaluate to 0
        // I add two ternary operators to figure out how much forward or backwards is being commanded via dpad
        double dpadForwardsVal = (driverPad.dpad_up ? .2 : 0 ) + (driverPad.dpad_down ? -.2 :0);
        double dpadSidewaysVal=(driverPad.dpad_right ? .2 : 0 ) + (driverPad.dpad_left ? -.2 :0);
        double bumperVal=(driverPad.left_bumper ? .4 : 0 ) + (driverPad.right_bumper ? -.4 :0);

        double P = Math.hypot(-driverPad.left_stick_x+bumperVal, -driverPad.left_stick_y+dpadForwardsVal);
        double robotAngle = Math.atan2(-driverPad.left_stick_y+dpadForwardsVal, -driverPad.left_stick_x+bumperVal);
        double rightX = driverPad.right_stick_x+dpadSidewaysVal;

        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v1 = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontRight
        final double v2 = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontLeft
        final double v3 = (P * sinRAngle) + (P * cosRAngle) + rightX;  //backRight
        final double v4 = (P * sinRAngle) - (P * cosRAngle) - rightX;  //backLeft

        motorPowers[0] = v1; motorPowers[1] = v2; motorPowers[2] = v3; motorPowers[3] = v4;

    }

    /*
     * Instead of using the gamepad like the previous one, it takes in values for each direction (sideways and forwards/backwards) with x and z.
     * The values passed in for x, z, and rot should be in a range from -1 to 1.
     * For x, negative means left and positive means right.
     * For z, negative means backward and positive means forward.
     * For rot, negative means counterclockwise and positive means clockwise.
     */
    public void determineMotorPowers(double x, double z, double rot) {
        //P is the power
        //robotAngle is the angle that the robot is in right now
        //rightX is the value that figures out the rotation that you want to go to

        double P = Math.hypot(-x, z);
        double robotAngle = Math.atan2(z, -x);
        double rightX = rot;

        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v1 = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontRight
        final double v2 = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontLeft
        final double v3 = (P * sinRAngle) + (P * cosRAngle) + rightX;  //back RIght
        final double v4 = (P * sinRAngle) - (P * cosRAngle) - rightX;  //backLeft

        motorPowers[0] = v1; motorPowers[1] = v2; motorPowers[2] = v3; motorPowers[3] = v4;
    }
    //same as determine motor powers but doesnt let any motor powers be greater than 1
    public void determineMotorPowersLimited(double x, double z, double rot) {
        //P is the power
        //robotAngle is the angle that the robot is in right now
        //rightX is the value that figures out the rotation that you want to go to

        double P = Math.hypot(-x, z);
        double robotAngle = Math.atan2(z, -x);
        double rightX = rot;
        // if the magnitude is greater than 1 we can just cap it and nothing in the angle or anything like that changes
        if (P>1){
            P=1;
        }
        //check and see if our motor powers will be greater than 1 when the stick comes into play,
        //not sure if this is sound and it needs testing
        //also prioritises rotation which may want to be changed
//        if(P+Math.abs(rightX)>1){
//            P=1-Math.abs(rightX);
//        }
        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v1 = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontRight
        final double v2 = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontLeft
        final double v3 = (P * sinRAngle) + (P * cosRAngle) + rightX;  //back RIght
        final double v4 = (P * sinRAngle) - (P * cosRAngle) - rightX;  //backLeft

        motorPowers[0] = v1; motorPowers[1] = v2; motorPowers[2] = v3; motorPowers[3] = v4;
    }

    /*
     * Like the previous determineMotorPowers methods, this method undergoes mecanum calculations
     *   to find the powers that shouold go to each wheel with its diagonal rollers.
     * However, this one makes driving easier with a "Pacman" sort of orientation that allows
     *   drivers to drive based on what's forward and backward in relation to them, not the robot.
     *   It does this with information from the odometry about the robot's rotation.
     */

    public void fieldRelDetermineMotorPowers(double x, double z, double rot) {
        //P is the power
        //robotAngle is the angle to which you want to go
        //rightX is the value of the x-axis from the right joystick

        double P = Math.hypot(-x, z);
        double robotAngle = Math.atan2(z, -x);
        double rightX = rot;

        double sinRAngle = Math.sin(robotAngle-Math.toRadians(robot.odometry.getPosition().getLocation(3)));
        double cosRAngle = 1.2*Math.cos(robotAngle-Math.toRadians(robot.odometry.getPosition().getLocation(3)));
//        telemetree.addData("robot angle",robotAngle);
//        telemetree.addData("sin angle",sinRAngle);
//        telemetree.addData("cos angle",cosRAngle);

        final double frPower = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontRight
        final double flPower = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontLeft
        final double brPower = (P * sinRAngle) + (P * cosRAngle) + rightX;  //backRight
        final double blPower = (P * sinRAngle) - (P * cosRAngle) - rightX;  //backLeft


        motorPowers[0] = frPower; motorPowers[1] = flPower; motorPowers[2] = brPower; motorPowers[3] = blPower;

    }

    /*
     * Stops the motors of the drivetrain!
     */
    public void stopDrivetrain(){
        determineMotorPowers(0,0,0);
        this.write();
    }

    /*
     * Returns the rotation error that PID loops can use.
     * goal is the rotation you want to be at, current is the rotation that the robot is currently
     * at.
     */
    public float rotationError(float goal, float current){
        spinError = goal/*where we want to be*/ - current/*where we are*/ ;

        if(spinError > 180) {
            spinError = spinError - 360;
        } else if (spinError < -180) {
            spinError = spinError + 360;
        }
        else if(spinError==180){
            spinError = previousSpinError;
        }
        return (float) spinError;
    }

    /*
     * Returns a location object that is the error for each value (x, z, and rot).
     * goalPos is where we want to be.
     * Goes through PID calculations to find the output for each motor.
     * Sets motor powers appropriately.
     */
    public Location findErrorCorrect(Location goalPos) {
        Location error = new Location(
                goalPos.getLocation(0)-robot.odometry.realMaybe.getLocation(0),
                0,
                goalPos.getLocation(2) - robot.odometry.realMaybe.getLocation(2),
                rotationError( goalPos.getLocation(3), robot.odometry.realMaybe.getLocation(3)));
        //this is to change the global xy error into robot specific error
        magnitude = Math.hypot(-error.getLocation(0),error.getLocation(2));
        robotheading = robot.odometry.getPosition().getLocation(3)- Math.atan2(error.getLocation(2),-error.getLocation(0));
        robotheading = Math.atan2(error.getLocation(0),error.getLocation(2));

        double forwardError = Math.cos(robotheading-Math.toRadians(robot.odometry.realMaybe.getLocation(3)))*magnitude;
        double strafeError = Math.sin(robotheading-Math.toRadians(robot.odometry.realMaybe.getLocation(3)))*magnitude;




        if(Math.abs(Variables.kfP*forwardError + Variables.kfI*integralValues[0] + Variables.kfD * (forwardError- lastForwardError))<1)
            integralValues[0]= integralValues[0]+forwardError ;
        if(Math.abs(Variables.ksP*strafeError + Variables.ksI*integralValues[2] + Variables.ksD * (strafeError - lastForwardError))<1)
            integralValues[2]= integralValues[2]+strafeError;
        if(Math.abs(Variables.krP*error.getLocation(3) + Variables.krI*integralValues[3] + Variables.krD * (error.getLocation(3) - lastForwardError))<1)
            integralValues[3]= integralValues[3]+error.getLocation(3);
        //fix
        //angle-robot
        //fix the problem when magnitude is greater than 1
        double forwardPow= Variables.kfP*forwardError+ Variables.kfI*integralValues[0] + Variables.kfD * (forwardError - lastForwardError);
        double sidePow= Variables.ksP*strafeError + Variables.ksI*integralValues[2] + Variables.ksD * (strafeError - lastSidewaysError);
        double rotPow= Variables.krP *error.getLocation(3) + Variables.krI*integralValues[3] +Variables.krD * ( error.getLocation(3) - lastRotationError);

        lastForwardError = forwardPow;
        lastSidewaysError = sidePow;
        lastRotationError = rotPow;


        determineMotorPowers(sidePow,forwardPow,rotPow);
        return error;
    }
    public Location findError(Location goalPos) {
        Location error = new Location(
                goalPos.getLocation(0)-robot.odometry.getPosition().getLocation(0),
                0,
                goalPos.getLocation(2) - (robot.odometry.getPosition().getLocation(2)),
                rotationError( goalPos.getLocation(3), robot.odometry.getPosition().getLocation(3)));
        //this is to change the global xy error into robot specific error
        double magnitude = Math.sqrt(Math.pow(error.getLocation(0),2)+ Math.pow(error.getLocation(2),2));
        double robotheading = robot.odometry.getPosition().getLocation(3)- Math.atan(error.getLocation(0)/error.getLocation(2));

        if(Math.abs(Variables.kfP*error.getLocation(0) + Variables.kfI*integralValues[0] + Variables.kfD * (error.getLocation(0) - lastForwardError))<1)
            integralValues[0]= integralValues[0]+error.getLocation(0) ;
        if(Math.abs(Variables.ksP*error.getLocation(2) + Variables.ksI*integralValues[2] + Variables.ksD * (error.getLocation(2) - lastForwardError))<1)
            integralValues[2]= integralValues[2]+error.getLocation(2);
        if(Math.abs(Variables.krP*error.getLocation(3) + Variables.krI*integralValues[3] + Variables.krD * (error.getLocation(3) - lastForwardError))<1)
            integralValues[3]= integralValues[3]+error.getLocation(3);
        //fix
        //angle-robot
        double forwardPow= Variables.kfP*error.getLocation(0) + Variables.kfI*integralValues[0] + Variables.kfD * (error.getLocation(0) - lastForwardError);
        double sidePow= Variables.ksP*error.getLocation(2) + Variables.ksI*integralValues[2] + Variables.ksD * ( error.getLocation(2) - lastSidewaysError);
        double rotPow= Variables.krP *error.getLocation(3) + Variables.krI*integralValues[3] + Variables.krD * ( error.getLocation(3) - lastRotationError);

        lastForwardError = forwardPow;
        lastSidewaysError = sidePow;
        lastRotationError = rotPow;

        double hypot = Math.sqrt(Math.pow(forwardPow,2)+Math.pow(sidePow,2));
        if(hypot>=1){
            forwardPow = forwardPow/hypot;
            sidePow = sidePow/hypot;
        }
        fieldRelDetermineMotorPowers(sidePow,forwardPow,rotPow);
        return error;
    }

    /*
     * Like move to position, but it only rotates.
     */
    public Location rotateToPosition(Location goalPos) {
        Location error = new Location(
                0,
                0,
                0,
                rotationError(goalPos.getLocation(3), robot.odometry.getPosition().getLocation(3)));

        if(Variables.krP*error.getLocation(3)<1)
            integralValues[3]= integralValues[3]+error.getLocation(3);

        double rotPow= Variables.krP*error.getLocation(3)+Variables.krI*integralValues[3];

        fieldRelDetermineMotorPowers(0,0,rotPow);
        return error;
    }

    /*
     * Sends power to the drivetrain for however long it takes for the robot to move to goal position passed in.
     * The other parameters are the tolerances, which show how accurate the movement needs to be.
     *    The tolerances should be passed in as millimeters.
     */
    public void actuallyMoveToPosition(Location goalPos, double xTolerance, double zTolerance, double rotTolerance) {
        integralValues = new double[4];
        error = findError(goalPos);
        while (Math.abs(error.getLocation(0)) > xTolerance || Math.abs(error.getLocation(2)) > zTolerance || Math.abs(error.getLocation(3)) > rotTolerance) {
            error = findError(goalPos);
            write();
            robot.odometry.updatePosition();
            telemetree.addData("Error", + error.getLocation(0) + ", " + error.getLocation(2) + ", " + error.getLocation(3));
            telemetree.addData("Location", robot.odometry.getPosition().getLocation(0) + " " + robot.odometry.getPosition().getLocation(2) + " " + robot.odometry.getPosition().getLocation(3));

            telemetree.update();
            dashboardTelemetry.addData("x-error",error.getLocation(0) );
            dashboardTelemetry.addData("y-error",error.getLocation(2) );
            dashboardTelemetry.addData("r-error",error.getLocation(3) );
            dashboardTelemetry.update();
        }
        stopDrivetrain();
        LinearOpMode op = (LinearOpMode) robot.oop;
        op.sleep(500);
    }
    public void actuallyMoveToPositionCorrect(Location goalPos, double xTolerance, double zTolerance, double rotTolerance) {
        integralValues = new double[4];
        error = findErrorCorrect(goalPos);
        while (robot.linoop.opModeIsActive()&&(Math.abs(error.getLocation(0)) > xTolerance || Math.abs(error.getLocation(2)) > zTolerance || Math.abs(error.getLocation(3)) > rotTolerance)) {
            error = findErrorCorrect(goalPos);
            write();
            robot.odometry.updatePosition();
            telemetree.addData("Error", + error.getLocation(0) + ", " + error.getLocation(2) + ", " + error.getLocation(3));
            telemetree.addData("Location", robot.odometry.getPosition().getLocation(0) + " " + robot.odometry.getPosition().getLocation(2) + " " + robot.odometry.getPosition().getLocation(3));

            telemetree.update();
            dashboardTelemetry.addData("x-error",error.getLocation(0) );
            dashboardTelemetry.addData("y-error",error.getLocation(2) );
            dashboardTelemetry.addData("r-error",error.getLocation(3) );
            // dashboardTelemetry.update();
        }
        stopDrivetrain();

    }
    public void actuallyMoveToPositionCorrect(Location goalPos, double xTolerance, double zTolerance, double rotTolerance,int maxTime) {
        integralValues = new double[4];
        error = findErrorCorrect(goalPos);
        double startTime = robot.getTimeMS();
        while ((robot.getTimeMS() - startTime < maxTime) && robot.linoop.opModeIsActive()&&(Math.abs(error.getLocation(0)) > xTolerance || Math.abs(error.getLocation(2)) > zTolerance || Math.abs(error.getLocation(3)) > rotTolerance)) {
            error = findErrorCorrect(goalPos);
            write();
            robot.odometry.updatePosition();
            telemetree.addData("Error", + error.getLocation(0) + ", " + error.getLocation(2) + ", " + error.getLocation(3));
            telemetree.addData("Location", robot.odometry.getPosition().getLocation(0) + " " + robot.odometry.getPosition().getLocation(2) + " " + robot.odometry.getPosition().getLocation(3));

            telemetree.update();
            dashboardTelemetry.addData("x-error",error.getLocation(0) );
            dashboardTelemetry.addData("y-error",error.getLocation(2) );
            dashboardTelemetry.addData("r-error",error.getLocation(3) );
            // dashboardTelemetry.update();
        }
        stopDrivetrain();

    }



    /*
     * Moves the robot to the position passed in with a freedom provided by the tolerances (mm).
     * This time, it has a maximum time in milliseconds that the method can run so that it won't
     *    waste too much time.
     */
    public void actuallyMoveToPosition(Location goalPos, double xTolerance, double zTolerance, double rotTolerance, int maxTime) {
        integralValues = new double[4];
        error = findError(goalPos);
        double startTime = robot.getTimeMS();
        while (robot.linoop.opModeIsActive()&&(robot.getTimeMS() - startTime < maxTime && (Math.abs(error.getLocation(0)) > xTolerance || Math.abs(error.getLocation(2)) > zTolerance || Math.abs(error.getLocation(3)) > rotTolerance))) {
            error = findError(goalPos);
            write();
            robot.odometry.updatePosition();
            telemetree.addData("Error", + error.getLocation(0) + ", " + error.getLocation(2) + ", " + error.getLocation(3));
            telemetree.addData("Location", robot.odometry.getPosition().getLocation(0) + " " + robot.odometry.getPosition().getLocation(2) + " " + robot.odometry.getPosition().getLocation(3));
            telemetree.update();

            dashboardTelemetry.addData("x-error",error.getLocation(0) );
            dashboardTelemetry.addData("y-error",error.getLocation(2) );
            dashboardTelemetry.addData("r-error",error.getLocation(3) );
            //dashboardTelemetry.update();
        }
        stopDrivetrain();
        LinearOpMode op = (LinearOpMode) robot.oop;
        op.sleep(500);
    }

    /*
     * Rotates the robot to the goal position within the tolerance passed in (mm).
     * This time, it has a maximum amount of time (milliseconds) that the method can take to run.
     */
    public void actuallyRotate(Location goalPos, double rotTolerance,double maxTime) {
        integralValues=new double[4];
        error = findError(goalPos);
        long startTime = robot.getTimeMS();
        while (Math.abs(error.getLocation(3)) > rotTolerance && robot.getTimeMS() - startTime < maxTime) {
            error = rotateToPosition(goalPos);
            write();
            robot.odometry.updatePosition();
            telemetree.addData("Error", + error.getLocation(0) + ", " + error.getLocation(2) + ", " + error.getLocation(3));
            telemetree.addData("Location", robot.odometry.getPosition().getLocation(0) + " " + robot.odometry.getPosition().getLocation(2) + " " + robot.odometry.getPosition().getLocation(3));

            telemetree.update();
            dashboardTelemetry.addData("x-error",error.getLocation(0) );
            dashboardTelemetry.addData("y-error",error.getLocation(2) );
            dashboardTelemetry.addData("r-error",error.getLocation(3) );
            //dashboardTelemetry.update();
        }
        stopDrivetrain();
        LinearOpMode op = (LinearOpMode) robot.oop;
        op.sleep(500);
    }

    public Location pidd(Location goalPos) {
        Location error = new Location(
                goalPos.getLocation(0)-robot.odometry.getPosition().getLocation(0),
                0,
                goalPos.getLocation(2) - (robot.odometry.getPosition().getLocation(2)),
                rotationError(goalPos.getLocation(3), robot.odometry.getPosition().getLocation(3)));

        //Making sure that this is a new loop/instance of when we're running this so that the integral can reset.
        if(robot.getTimeMS() - timeWhenLeave > 200) {
            timeWhenLeave = robot.getTimeMS();
        }

        double timeElapsedSec = (robot.getTimeMS() - timeWhenLeave)/1000;

        if(Math.abs(Variables.wfP*error.getLocation(0) + Variables.wfI*integralValues[0] + Variables.wfD * (error.getLocation(0) - lastForwardError))<1)
            integralValues[0] = integralValues[0] + error.getLocation(0) * timeElapsedSec;
        if(Math.abs(Variables.wsP*error.getLocation(0) + Variables.wsI*integralValues[0] + Variables.wsD * (error.getLocation(0) - lastForwardError))<1)
            integralValues[2] = integralValues[2] + error.getLocation(2) * timeElapsedSec;
        if(Math.abs(Variables.wrP*error.getLocation(0) + Variables.wrI*integralValues[0] + Variables.wrD * (error.getLocation(0) - lastForwardError))<1)
            integralValues[3] = integralValues[3] + error.getLocation(3) * timeElapsedSec;

        double forwardPow= Variables.wfP*error.getLocation(0) + Variables.wfI*integralValues[0] + Variables.wfD * (error.getLocation(0) - lastForwardError);
        double sidePow= Variables.wsP*error.getLocation(2) + Variables.wsI*integralValues[2] + Variables.wsD * ( error.getLocation(2) - lastSidewaysError);
        double rotPow= Variables.wrP *error.getLocation(3) + Variables.wrI*integralValues[3] + Variables.wrD * ( error.getLocation(3) - lastRotationError);

        lastForwardError = forwardPow;
        lastSidewaysError = sidePow;
        lastRotationError = rotPow;

        double hypot = Math.sqrt(Math.pow(forwardPow,2)+Math.pow(sidePow,2));
        if(hypot>=1){
            forwardPow = forwardPow/hypot;
            sidePow = sidePow/hypot;
        }
        fieldRelDetermineMotorPowers(sidePow,forwardPow,rotPow);
        return error;
    }


    @Override
    public void update(Gamepad gp1, Gamepad gp2) {

        //determineMotorPowers(gp1);

        if (testing) {
            if(gp1.b) {
                actuallyMoveToPositionCorrect(Variables.shootPos,Variables.xTolernce, Variables.yTolerance, Variables.rotTolerance);
            }

        }
//un comment soon
        if(gp1.dpad_up){ //precision movement forward, very slow
            determineMotorPowers(0,0.2,0);
        }

        if(gp1.dpad_down){ //precision movement backward, very slow
            determineMotorPowers(0,-0.2,0);
        }

//            if(gp1.x) { //Automatic movement to the shooting position in teleOp
//                Location error1 = findError(new Location((float) Variables.targetx,0, (float) Variables.targetz,(float) Variables.targetRot));
//                dashboardTelemetry.addData("x-error",error1.getLocation(0) );
//                dashboardTelemetry.addData("z-error",error1.getLocation(2) );
//                dashboardTelemetry.addData("r-error",error1.getLocation(3) );
//                dashboardTelemetry.addData("inti-error",integralValues[3] );
//                dashboardTelemetry.update();
//            }


//        addPublicTelemetry("odo", odo.realMaybe.toString());
//        addPublicTelemetry("first time for shot", ""+firstTimeForShot);

        //addPublicTelemetry("pop pop" ,""+magnitude);
        // addPublicTelemetry("robot heading" ,""+robotheading);
    }

    @Override
    public void write() {
        //Sets the motor powers based on the determineMotorPowers() method that was run in the update() method
        int i = 0;
        for(DcMotorEx motor:motors.subList(dtMotorNumbers[0], dtMotorNumbers[3] + 1)){
            motor.setPower(motorPowers[i]);
            i++;
        }

    }
    public void odoUp () {
        servos.get(0).setPosition(0.27);
        servos.get(1).setPosition(0.32);
        servos.get(2).setPosition(0.57);
    }

    public void odoDown () {
        servos.get(0).setPosition(0.46);
        servos.get(1).setPosition(0.61);
        servos.get(2).setPosition(0.31);
    }
}