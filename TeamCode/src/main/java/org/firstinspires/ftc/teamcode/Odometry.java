package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

/**
 * Tracks the position of the robot
 * @author Jack 2
 *
 */
public class Odometry extends Mechanism {

    //VERY IMPORTANT, LAST YEAR'S ROBOT STATS, CHANGE NEW ONE BELOW FOR NEW BOT

//    //Declares constants that relate to odometry wheels
//    //Diameter of the encoders
//    private static final double ODO_DIAMETER_MM = 44.45;
//    //Number of ticks on the encoders
//    private static final double ODO_ENCODER_TICKS = 8192;
//    //Distance between odometry encoders
//    private static final double ODO_DISTANCE_MM = 411.1625;
//    //Circumference of the encoder
//    //Distance from the center encoder to the center of the robot
//    private static final double ODO_DISTANCE_FROM_CENTER = 53.975;

    //This year's robot odometry stats

    //Declares constants that relate to odometry wheels
    //Diameter of the encoders
    private static final double ODO_DIAMETER_MM = 35;
    //Gear ratio of the odometry wheels
    private static final double ODO_GEAR_RATIO = 2.6667;
    //Effective diameter of the odo wheels based on the gear ratio
    private static final double ODO_DIAMETER_EFFECTIVE_MM = ODO_DIAMETER_MM * ODO_GEAR_RATIO;
    //Number of ticks on the encoders
    private static final double ODO_ENCODER_TICKS = 8192;
    //Distance between odometry encoders
    private static final double ODO_DISTANCE_MM = 403.225;
    //Circumference of the encoder
    private static final double ODO_CIRCUMFERENCE_MM = ODO_DIAMETER_EFFECTIVE_MM * Math.PI;
    //Distance from the center encoder to the center of the robot
    private static final double ODO_DISTANCE_FROM_CENTER = 44.45;
    //The number of encoder ticks per millimeter
    private static final double ENCODER_TICKS_PER_MM = ODO_ENCODER_TICKS / ODO_CIRCUMFERENCE_MM;

    //Expansion hub data for the encoders
    /**Declares the first expansion hub*/
    private final ExpansionHubEx expansionHub;
    /**Declares an object that stores all of the static data*/
    public RevBulkData bulkData;

    //Current position fields
    /**Declares a new Location object to track position*/
    private Location position = new Location();
    /**Declares another new Location object to track position*/
    public Location pos2 = new Location();
    /**X position relative to starting location*/
    public double relativeX;
    /**Y position relative to starting location*/
    public double relativeY;
    /**Offset of the rotation*/
    private float rotOffset=0;
    /**Declares an array of encoder positions*/
    private int[] encoderPosition = new int[3];
    /**Declares an array of the offset for each encoder*/
    private int[] encoderPositionoffset = new int[3];

    /**Declares an array of encoder values*/
    public double[] encoderDeltamm = new double[3];


    public Location realMaybe = new Location();

    public Drivetrain drivetrain;



    /* *************************** ODOMETRY CONSTRUCTOR METHODS *************************** */

    /**
     * Odometry Constructor
     * @param hardwareMap
     */
    public Odometry(HardwareMap hardwareMap) {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        reset();

    }

    /**
     * Odometry Constructor
     * @param hardwareMap
     * @param startPos Starting position of the robot
     */
    public Odometry(HardwareMap hardwareMap, Location startPos) {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        reset(startPos);
    }

    /**
     * Odometry constructor
     * @param hardwareMap
     * @param distance
     * @param centerDistance
     * @param startingLocation
     */
    //New odometry constructor for new robot! The distance and distance from center will be different
    public Odometry(HardwareMap hardwareMap, double distance, double centerDistance,Location startingLocation) {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        reset(startingLocation);
        //ODO_DISTANCE_MM = 414.25;
        //ODO_DISTANCE_FROM_CENTER = -56.92; //-88.42
        //ODO_CIRCUMFERENCE_MM = Math.PI * ODO_DIAMETER_MM;
        //ENCODER_TICKS_PER_MM = ODO_ENCODER_TICKS / ODO_CIRCUMFERENCE_MM;
    }

    /**
     * Odometry constructor
     * @param hardwareMap
     * @param distance
     * @param centerDistance
     */
    public Odometry(HardwareMap hardwareMap, double distance, double centerDistance) {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        reset();
        //ODO_DISTANCE_MM = 420.478;
        //ODO_DISTANCE_FROM_CENTER = -56.92; //-88.42
        //ODO_CIRCUMFERENCE_MM = Math.PI * ODO_DIAMETER_MM;
        //ENCODER_TICKS_PER_MM = ODO_ENCODER_TICKS / ODO_CIRCUMFERENCE_MM;
    }

    /* *************************** RESET METHODS *************************** */

    /**
     * Resets methods and the robot's position, either to whatever argument
     * is/arguments are passed in, or, in the case of no arguments, to (0, 0, 0),
     * also resets the encoder positions.
     */
    public void reset() {
        try {
            bulkData = expansionHub.getBulkInputData();

            for (int i = 0; i < 3; i++) {
                if (i == 2) {
                    encoderPositionoffset[i] = -bulkData.getMotorCurrentPosition(i);
                }
                encoderPositionoffset[i] = bulkData.getMotorCurrentPosition(i);
            }
            rotOffset = 0;
            encoderPosition = new int[3];
            position.setLocation(0, 0, 0, 0);
        } catch (NullPointerException e) {

        }
    }

    /**
     * Resets methods and the robot's position, either to whatever argument is/arguments are passed
     * in, or, in the case of no arguments, to (0, 0, 0), also resets the encoder positions.
     * @param x The x coordinate for the robot to reset to
     * @param z The z coordinate that the robot needs to reset to
     * @param rot   The rotation angle
     */
    public void reset(float x, float z, float rot) {
        try {
            bulkData=expansionHub.getBulkInputData();
            for(int i=0;i<3;i++) {
                if (i == 2) {
                    encoderPositionoffset[i] = -bulkData.getMotorCurrentPosition(i);
                }
                encoderPositionoffset[i] = bulkData.getMotorCurrentPosition(i) ;
            }
            Location resettiSpot = new Location(x, 0, z, rot);
            encoderPosition = new int[3];
            position = resettiSpot;
            rotOffset = resettiSpot.getLocation(3);
        } catch (NullPointerException e) {

        }
    }

    /**
     * Resets methods and the robot's position, either to whatever argument is/arguments are passed
     * in, or, in the case of no arguments, to (0, 0, 0), also resets the encoder positions.
     * @param resetPos  a  position for the robot to reset to
     */
    public void reset(Location resetPos) {
        try {
            bulkData = expansionHub.getBulkInputData();
            for (int i = 0; i < 3; i++) {
                if (i == 2) {
                    encoderPositionoffset[i] = -bulkData.getMotorCurrentPosition(i);
                }
                encoderPositionoffset[i] = bulkData.getMotorCurrentPosition(i);
            }
            encoderPosition = new int[3];
            // if you are running into issues with this method this may be the cause
            // I have to reset a little backwards because of the jank way the odo is switched for backawards compatability
            position = resetPos;
            pos2=new Location(resetPos.getLocation(2),0,resetPos.getLocation(0),resetPos.getLocation(3));

            rotOffset = resetPos.getLocation(3);
        } catch (NullPointerException e) {

        }
    }

    /*
     Update Position Method
     Updates position to where the robot currently is. Counter-clockwise rotation is negative.
     Lots of math exists here, we have to take into account the encoder positions
     and how they've changed since the previous read.
     */
    public void updatePosition() {
        try {
            bulkData = expansionHub.getBulkInputData();
            for (int i = 0; i < 3; i++) {//updates the array encoderDeltamm for each odo wheel to see how much they've moved in mm
                //if and else for the different expansion hubs... also because the two wheels facing forward need to have negative bulk data reads
                if (i == 2 || (i == 1 && ODO_DISTANCE_MM != 375.1)) {
                    encoderDeltamm[i] = -ODO_CIRCUMFERENCE_MM * ((bulkData.getMotorCurrentPosition(i) - encoderPositionoffset[i] + encoderPosition[i]) / ODO_ENCODER_TICKS);
                    encoderPosition[i] = -bulkData.getMotorCurrentPosition(i) + encoderPositionoffset[i];
                } else {
                    //normal non reversed case
                    encoderDeltamm[i] = ODO_CIRCUMFERENCE_MM * ((bulkData.getMotorCurrentPosition(i) - encoderPositionoffset[i] - encoderPosition[i]) / ODO_ENCODER_TICKS);
                    encoderPosition[i] = bulkData.getMotorCurrentPosition(i) - encoderPositionoffset[i];
                }
            }

            double botRotDelta = (encoderDeltamm[0] - encoderDeltamm[1]) / ODO_DISTANCE_MM;  //finds change in robo rotation
            relativeX = encoderDeltamm[2] + (ODO_DISTANCE_FROM_CENTER * botRotDelta); //strafing distance
            relativeY = (encoderDeltamm[0] + encoderDeltamm[1]) / 2; //how much moved forward/back
            //setting current robo rotation in Location object
//            position.setRotation((float) Math.toDegrees(((ODO_CIRCUMFERENCE_MM/*circumference*/ * ((encoderPosition[0]) / ODO_ENCODER_TICKS)/*percentage of the wheel revolved*/ - (ODO_CIRCUMFERENCE_MM * ((encoderPosition[1]) / ODO_ENCODER_TICKS)))) / ODO_DISTANCE_MM));
            double angle = (float) Math.toDegrees((encoderPosition[1] - encoderPosition[0]) / (ODO_DISTANCE_MM * ENCODER_TICKS_PER_MM));
            angle=angle+rotOffset;
            position.setRotation(angle);
            if (Math.abs(botRotDelta) > 0) {
                double radiusOfMovement = (encoderDeltamm[0] + encoderDeltamm[1]) / (2 * botRotDelta); //Radius that robot moves around
                double radiusOfStraif = relativeX / botRotDelta; //radius of the robot while also moving forward :O

                relativeY = (radiusOfMovement * Math.sin(botRotDelta)) - (radiusOfStraif * (1 - Math.cos(botRotDelta)));

                relativeX = radiusOfMovement * (1 - Math.cos(botRotDelta)) + (radiusOfStraif * Math.sin(botRotDelta));
                addPublicTelemetry("relativex ", ""+relativeX);
            }
            position.translateLocal(relativeY, relativeX, 0);
            pos2.setLocation(position.getLocation(2), position.getLocation(1), position.getLocation(0), position.getLocation(3));
        } catch (NullPointerException e) {
        }
        drivetrain.telemetree.addData("",""+encoderDeltamm[0]);
        drivetrain.telemetree.addData("",""+encoderDeltamm[1]);
        drivetrain.telemetree.addData("",""+encoderDeltamm[2]);
    }

    /* *************************** GETTER METHODS *************************** */

    /**
     * Get's the position of the encoder
     * @return encoderPosition  the position of the encoder
     */
    public int[] getEncoderPosition() {return encoderPosition;}

    /**
     * Gets the position
     * @return Location or pos
     */
    public Location getPosition() {
        try {return position;}
        catch (NullPointerException e) {
            return new Location();
        }
    }

    /* *************************** TELEMETRY STRING METHODS *************************** */

    /**
     * Converts the encoder position to a string
     */
    public String currentEncoderPosString() {return encoderPosition[0] + ", " + encoderPosition[1] + ", " + encoderPosition[2];}

    /**
     * Converts the robot position to a string
     */
    public String currentRobotPositionString() {return position.getLocation(0) + ", " + position.getLocation(2) + ", " + position.getLocation(3);}

    /* *************************** GETTER METHODS *************************** */

    /**
     * Updates every cycle
     */
    @Override
    public void update(Gamepad gp1, Gamepad gp2) {

    }

    /**
     * Updates every cycle
     */
    @Override
    public void write() {

    }
}
