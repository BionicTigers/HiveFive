package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;

import com.qualcomm.robotcore.hardware.Servo;

/*
This class declares the drivetrain mechanism and sends data from the controller to the robot and
 uses that data to set the motor powers
 */

public class Drivetrain extends Mechanism {
    //Declares values
    public org.firstinspires.ftc.teamcode.Robot robot;
    public double[] motorPowers;
    public int[] motorIndices;
    public Telemetry telemetree;
    //private Odometry odo;
    private Location forward = new Location (0, 0, 1000, 0);
    private Location backward = new Location (0, 0, -1000, 0);
    private Location left = new Location (-500, 0, 0, 0);
    private Location right = new Location (500, 0, 0, 0);
    private Location clockwise = new Location (0, 0, 0, 90);
    private Location counterclockwise = new Location (0, 0, 0, 270);
    private Location center = new Location (0, 0, 0, 0);
    //Spin PID variables
    public double spinError;
    public double previousSpinError=20;
    public double timeWhenLeave;


    private double lastForwardError;
    private double lastSidewaysError;
    private double lastRotationError;
    public boolean autoIsDone = false;

    private Telemetry dashboardTelemetry;

    public Location error = new Location();

    public double[] integralValues=new double[4];

    public double sinrang = 0;
    public double cosrang = 0;
    public double pow = 0;


    //Constructor method
    public Drivetrain(@NonNull org.firstinspires.ftc.teamcode.Robot bot, @NonNull int[] motorNumbers, Telemetry T, Servo servo, Servo servo2, Servo servo3) {
        DcMotorEx motorPlaceholder;
        robot = bot;
        motorIndices = motorNumbers;
        telemetree = T;
        //odo = bot.odometry;
        getServos().add(servo);

        for (int motNum : motorNumbers) {
            motorPlaceholder = robot.motors.get(motNum);
            motorPlaceholder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.add(motorPlaceholder);
        }
        motorPowers = new double[]{0, 0, 0, 0};
    }
    //sets the motorNumbers array based on input from joysticks
    public void determineMotorPowers (Gamepad driverPad){
        double P = Math.hypot(-driverPad.left_stick_x, -driverPad.left_stick_y);
        double robotAngle = Math.atan2(-driverPad.left_stick_y, -driverPad.left_stick_x);
        double rightX = driverPad.right_stick_x;

        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v1 = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontRight
        final double v2 = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontLeft
        final double v3 = (P * sinRAngle) + (P * cosRAngle) + rightX;  //backRight
        final double v4 = (P * sinRAngle) - (P * cosRAngle) - rightX;  //backLeft

        motorPowers[0] = v1;
        motorPowers[1] = v2;
        motorPowers[2] = v3;
        motorPowers[3] = v4;
    }
    public void determineServoMovement(Gamepad driverPad){
        if (driverPad.a){
            servos.get(0).setPosition(0.46);
            servos.get(1).setPosition(0.6);
            servos.get(2).setPosition(0.6);

        } else if(driverPad.b){
            servos.get(0).setPosition(0.3);
            servos.get(1).setPosition(0.67);
            servos.get(2).setPosition(0.67);
        }
    }
    //Uses values from motor powers array to move the robot
    public void robotMovement () {

    }
    public void update (Gamepad gp1, Gamepad gp2){
            /*
            All telemetree commands implement telemetry to allow the driver to view motor powers
            while code is active
             */
        telemetree.addLine("Motor Powers");
        telemetree.addData("Front Right Power", motorPowers[0]);
        telemetree.addData("Front Left Power", motorPowers[1]);
        telemetree.addData("Back Right Power", motorPowers[2]);
        telemetree.addData("Back Left Power", motorPowers[3]);
        telemetree.update();
        determineMotorPowers(gp1); //Updates values in motorPowers array
        determineServoMovement(gp1);
    }

    public void write () {
        //Sets the motor powers based on the determineMotorPowers() method that was run in the update() method
        int i = 0;
        for (DcMotorEx motor : motors.subList(motorIndices[0], motorIndices[3] + 1)) {
            motor.setPower(motorPowers[i]);
            i++;
        }
    }

    public void actuallyMoveToPosition(Location goalPos, double xTolerance, double zTolerance, double rotTolerance, int maxTime) {
        integralValues = new double[4];
        error = findError(goalPos);
        double startTime = robot.getTimeMS();
        while (robot.Linoop.opModeIsActive()&&(robot.getTimeMS() - startTime < maxTime && (Math.abs(error.getLocation(0)) > xTolerance || Math.abs(error.getLocation(2)) > zTolerance || Math.abs(error.getLocation(3)) > rotTolerance))) {
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
        if(Math.abs(Variables.krp*error.getLocation(3) + Variables.krI*integralValues[3] + Variables.krD * (error.getLocation(3) - lastForwardError))<1)
            integralValues[3]= integralValues[3]+error.getLocation(3);
        //fix
        //angle-robot
        double forwardPow= Variables.kfP*error.getLocation(0) + Variables.kfI*integralValues[0] + Variables.kfD * (error.getLocation(0) - lastForwardError);
        double sidePow= Variables.ksP*error.getLocation(2) + Variables.ksI*integralValues[2] + Variables.ksD * ( error.getLocation(2) - lastSidewaysError);
        double rotPow= Variables.krp *error.getLocation(3) + Variables.krI*integralValues[3] + Variables.krD * ( error.getLocation(3) - lastRotationError);

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

    public void stopDrivetrain(){
        determineMotorPowers(0,0,0);
        this.write();
    }

    public void determineMotorPowers(double x, double z, double rot) {
        //P is the power
        //robotAngle is the angle that the robot is in right now
        //rightX is the value that figures out the rotation that you want to go to

        double P = Math.hypot(-x, z);
        double robotAngle = Math.atan2(z, -x);
        double rightX = rot;

        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);
        cosrang = cosRAngle;
        sinrang = sinRAngle;
        pow = P;

        final double v1 = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontRight
        final double v2 = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontLeft
        final double v3 = (P * sinRAngle) + (P * cosRAngle) + rightX;  //back RIght
        final double v4 = (P * sinRAngle) - (P * cosRAngle) - rightX;  //backLeft

        motorPowers[0] = v1; motorPowers[1] = v2; motorPowers[2] = v3; motorPowers[3] = v4;
    }
}
