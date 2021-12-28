package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class declares the drivetrain mechanism and sends data from the controller to the robot and uses that data to set the motor powers
 */
public class Drivetrain extends Mechanism {
    //Declares variables
    /**declares a new instance of Robot*/
    public Robot robot;
    /**declares an array of motor powers*/
    public double[] motorPowers;
    /**declares a new array of motor indices*/
    public int[] motorIndices;
    /**declares a new instance of Telemetry*/
    public Telemetry telemetry;

    public PIDloops loops;

    public Location location;

    private double robotheading;
    private double magnitude;

    //Locations
    /**declares an instance of Location to move the robot forward*/
    private Location forward = new Location (0, 0, 1000, 0);
    /**declares an instance of Location to move the robot backward*/
    private Location backward = new Location (0, 0, -1000, 0);
    /**declares an instance of Location to move the robot left*/
    private Location left = new Location (-500, 0, 0, 0);
    /**declares an instance of Location to move the robot back*/
    private Location right = new Location (500, 0, 0, 0);
    /**declares an instance of Location to move the robot clockwise*/
    private Location clockwise = new Location (0, 0, 0, 90);
    /**declares an instance of Location to move the robot counterclockwise*/
    private Location counterclockwise = new Location (0, 0, 0, 270);
    /**Creates an instance of Location to move the robot to the center*/
    private Location center = new Location (0, 0, 0, 0);

    //Spin PID variables
    /**the error of a spin*/
    public double spinError;
    /**the last spin error*/
    public double previousSpinError=20;
    /**?*/
    public double timeWhenLeave;

    /**the last forward error*/
    private double lastForwardError;
    /**the last sideways error*/
    private double lastSidewaysError;
    /**the last rotation error*/
    private double lastRotationError;
    /**boolean that shows if auto is finished*/
    public boolean autoIsDone = false;

    /**declares a new instance of location to store x y and z errors*/
    public Location error = new Location();

    /**?*/
    public double[] integralValues=new double[4];

    /**?*/
    public double sinrang = 0;
    /**?*/
    public double cosrang = 0;
    /**motor powers*/
    public double pow = 0;


    /**
     * Constructs a drivetrain object
     * @param bot a new instance of Robot
     * @param motorNumbers motor data
     * @param T declares a new instance of Telemetry
     */
    public Drivetrain(@NonNull org.firstinspires.ftc.teamcode.Robot bot, @NonNull int[] motorNumbers, Telemetry T, Servo SDrive1, Servo SDrive2, Servo SDrive3) {
        DcMotorEx motorPlaceholder;
        robot = bot;
        motorIndices = motorNumbers;
        telemetry = T;
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

    /**
     * Sets the motorNumbers array based on input from joysticks
     * @param driverPad gamepad used to control the robot
     */
    public void determineMotorPowers (Gamepad driverPad){
        double dpadVal=0;
        if (driverPad.dpad_right)
            dpadVal=.2;
        if (driverPad.dpad_left)
            dpadVal=-.2;

        double P = Math.hypot(-driverPad.left_stick_x, -driverPad.left_stick_y);
        double robotAngle = Math.atan2(-driverPad.left_stick_y, -driverPad.left_stick_x);
        double rightX = driverPad.right_stick_x+dpadVal;

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

    /**
     * Updates data for Telemetry, motor powers, and servo movements
     * @param gp1 first gamepad
     * @param gp2 second gamepad
     */
    public void update (Gamepad gp1, Gamepad gp2){
        determineMotorPowers(gp1); //Updates values in motorPowers array

        if (gp1.b) {
            odoUp();
        } else if (gp1.a) {
            odoDown();
        }

        if(gp1.dpad_up){ //precision movement forward, very slow
            determineMotorPowers(0,0.2,0);
        }

        if(gp1.dpad_down){ //precision movement backward, very slow
            determineMotorPowers(0,-0.2,0);
        }
    }

    /**
     * Sets the motor powers based on the determineMotorPowers() method that was run in the update() method
     */
    public void write () {
        int i = 0;
        for (DcMotorEx motor : motors.subList(motorIndices[0], motorIndices[3] + 1)) {
            motor.setPower(motorPowers[i]);
            i++;
        }
        robot.odometry.updatePosition();
        //Sets all telemetry for the drivetrain
        telemetry.addLine("Motor Powers");
        telemetry.addData("Front Right Power", motorPowers[0]);
        telemetry.addData("Front Left Power", motorPowers[1]);
        telemetry.addData("Back Right Power", motorPowers[2]);
        telemetry.addData("Back Left Power", motorPowers[3]);
        telemetry.addData("Error", + error.getLocation(0) + ", " + error.getLocation(2) + ", " + error.getLocation(3));
        telemetry.addData("Location", robot.odometry.getPosition().getLocation(0) + " " + robot.odometry.getPosition().getLocation(2) + " " + robot.odometry.getPosition().getLocation(3));
        telemetry.addData("Odo wheel 1", error.getLocation(0));
        telemetry.addData("Odo wheel 2", error.getLocation(2));
        telemetry.addData("Odo wheel 3", error.getLocation(3));

        telemetry.update();
    }

    /**
     * Moves to robot to goalPos in maxTime
     * @param goalPos   the final position of the robot
     * @param xTolerance    the tolerance for the x coordinate
     * @param zTolerance    the tolerance for the z coordinate
     * @param rotTolerance  tolerance for the rotation
     * @param maxTime   maximum amount of time that the robot can take
     */
    public void moveToPosition(Location goalPos, double xTolerance, double zTolerance, double rotTolerance, int maxTime) {
        integralValues = new double[4];
        error = findError(goalPos);
        double startTime = robot.getTimeMS();
        //While the op mode is active, max time has not been reached, and error is within the x tolerance, error is within the z tolerance, or error is within the rotation tolerance
        while (robot.linoop.opModeIsActive() && (robot.getTimeMS() - startTime < maxTime && (Math.abs(error.getLocation(0)) > xTolerance || Math.abs(error.getLocation(2)) > zTolerance || Math.abs(error.getLocation(3)) > rotTolerance))) {
            //Finds the position error
            error = findError(goalPos);

//            loops.determineError(error);

            //Position is updated
            robot.odometry.updatePosition();

            //Write is called
            write();
        }
        stopDrivetrain();
        //op is set to robot.oop
        LinearOpMode op = (LinearOpMode) robot.linoop;
        //op sleeps for 500 ms
        op.sleep(500);
    }

    /**
     * Finds location error
     * @param goalPos the final position of the robot
     * @return the distance from the goalPos
     */
    public Location findError(Location goalPos) {
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
        // fix the problem when magnitude is greater than 1
        double forwardPow= Variables.kfP*forwardError+ Variables.kfI*integralValues[0] + Variables.kfD * (forwardError - lastForwardError);
        double sidePow= Variables.ksP*strafeError + Variables.ksI*integralValues[2] + Variables.ksD * (strafeError - lastSidewaysError);
        double rotPow= Variables.krP*error.getLocation(3) + Variables.krI*integralValues[3] +Variables.krD * ( error.getLocation(3) - lastRotationError);

        lastForwardError = forwardPow;
        lastSidewaysError = sidePow;
        lastRotationError = rotPow;


        determineMotorPowers(sidePow,forwardPow,rotPow);
        return error;
    }

    /**
     * Determines powers for each motor
     * @param x x coordinate
     * @param z z coordinate
     * @param rot needed rotation
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
//        telemetry.addData("robot angle",robotAngle);
//        telemetry.addData("sin angle",sinRAngle);
//        telemetry.addData("cos angle",cosRAngle);

        final double frPower = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontRight
        final double flPower = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontLeft
        final double brPower = (P * sinRAngle) + (P * cosRAngle) + rightX;  //backRight
        final double blPower = (P * sinRAngle) - (P * cosRAngle) - rightX;  //backLeft


        motorPowers[0] = frPower; motorPowers[1] = flPower; motorPowers[2] = brPower; motorPowers[3] = blPower;

    }

    /**
     * Calculates the error for the rotation
     * @param goal  where we need to rotate to
     * @param current where we are
     * @return how far off the rotation was
     */
    public float rotationError(float goal, float current){
        spinError = goal - current ;

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

    /**
     * Stops the drivetrain
     */
    public void stopDrivetrain(){
        determineMotorPowers(0,0,0);
        this.write();
    }

    /**
     * Determines motor powers
     * @param x final x coordinate
     * @param z final z coordinate
     * @param rot final rotation
     */
    public void determineMotorPowers(double x, double z, double rot) {
        //Power
        double P = Math.hypot(-x, z);
        //The angle that the robot is in right now
        double robotAngle = Math.atan2(z, -x);
        //The value that figures out the rotation that you want to go to
        double rightX = rot;

        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);
        cosrang = cosRAngle;
        sinrang = sinRAngle;
        pow = P;

        final double v1 = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontRight
        final double v2 = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontLeft
        final double v3 = (P * sinRAngle) + (P * cosRAngle) + rightX;  //backRight
        final double v4 = (P * sinRAngle) - (P * cosRAngle) - rightX;  //backLeft

        motorPowers[0] = v1; motorPowers[1] = v2; motorPowers[2] = v3; motorPowers[3] = v4;
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