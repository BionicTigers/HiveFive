package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.AutoStuff.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.AutoStuff.RevBulkData;
import org.firstinspires.ftc.teamcode.AutoStuff.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.AutoStuff.RevBulkData;

/**
 * Tracks the position of the robot
 * @author Jack 2
 *
 */
public class Odometry2 extends Mechanism {
    //Declares constants that relate to odometry wheels
    /**Diameter of the encoders*/
    private static final double ODO_DIAMETER_MM = 87.5;
    /**Number of ticks on the encoders*/
    private static final double ODO_ENCODER_TICKS = 8192;
    /**Distance between odometry encoders*/
    private static final double ODO_DISTANCE_MM = 402.085;
    /**Circumference of the encoder*/
    private static final double ODO_CIRCUMFERENCE_MM = ODO_DIAMETER_MM * Math.PI;
    /**Distance from the center encoder to the center of the robot*/
    private static final double ODO_DISTANCE_FROM_CENTER = 38.1; //CHANGE!!!
    /**The number of encoder ticks per millimeter*/
    private static final double ENCODER_TICKS_PER_MM = ODO_ENCODER_TICKS / ODO_CIRCUMFERENCE_MM;

    //Expansion hub data for the encoders
    /**Declares the first expansion hub*/
    private final ExpansionHubEx expansionHub;
    /**Declares the second expansion hub*/
    private final ExpansionHubEx expansionHub2;
    /**Declares an object that stores all of the static data*/
    private RevBulkData bulkData;

    //Current position fields
    /**Declares a new Location object to track position*/
    private Location pos = new Location();
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

    /* *************************** ODOMETRY CONSTRUCTOR METHODS *************************** */

    /**
     * Odometry Constructor
     * @param hardwareMap
     */
    public Odometry2(HardwareMap hardwareMap) {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 173");
        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 7");

        reset();

    }

    /**
     * Odometry Constructor
     * @param hardwareMap
     * @param startPos Starting position of the robot
     */
    public Odometry2(HardwareMap hardwareMap, Location startPos) {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 173");
        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 7");

        reset(startPos);
    }

    /**
     * Odometry constructor
     * @param hardwareMap the hardware map of the robot
     * @param distance distance the robot hsa moved
     * @param centerDistance distance from the center
     * @param startingLocation starting location of the robot
     */
    //New odometry constructor for new robot! The distance and distance from center will be different
    public Odometry2(HardwareMap hardwareMap, double distance, double centerDistance,Location startingLocation) {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 173");
        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 7");
        reset(startingLocation);
        //ODO_DISTANCE_MM = 414.25;
        //ODO_DISTANCE_FROM_CENTER = -56.92; //-88.42
        //ODO_CIRCUMFERENCE_MM = Math.PI * ODO_DIAMETER_MM;
        //ENCODER_TICKS_PER_MM = ODO_ENCODER_TICKS / ODO_CIRCUMFERENCE_MM;
    }

    /**
     * Odometry constructor
     * @param hardwareMap the hardware map of the robot
     * @param distance distance the robot has moved
     * @param centerDistance distance from the center of the field
     */
    public Odometry2(HardwareMap hardwareMap, double distance, double centerDistance) {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 173");
        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 7");
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
            pos.setLocation(0, 0, 0, 0);
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
            pos = resettiSpot;
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
            pos = resetPos;
            pos2=new Location(resetPos.getLocation(2),0,resetPos.getLocation(0),resetPos.getLocation(3));

            rotOffset = resetPos.getLocation(3);
        } catch (NullPointerException e) {

        }
    }

    /**
     * Update Position Method
     * Updates position to where the robot currently is. Counter-clockwise rotation is negative.
     * Loooooots of math exists here, we have to take into account the encoder positions and how
     * they've changed since the previous read.
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
            addPublicTelemetry("",""+encoderDeltamm[0]);
            addPublicTelemetry("",""+encoderDeltamm[1]);
            addPublicTelemetry("",""+encoderDeltamm[2]);


            double botRotDelta = (encoderDeltamm[0] - encoderDeltamm[1]) / ODO_DISTANCE_MM;  //finds change in robo rotation
            relativeX = encoderDeltamm[2] + (ODO_DISTANCE_FROM_CENTER * botRotDelta); //strafing distance
            relativeY = (encoderDeltamm[0] + encoderDeltamm[1]) / 2; //how much moved forward/back
            /*setting current robo rotation in Location object*/
            // pos.setRotation((float) Math.toDegrees(((ODO_CIRCUMFERENCE_MM/*circumference*/ * ((encoderPosition[0]) / ODO_ENCODER_TICKS)/*percentage of the wheel revolved*/ - (ODO_CIRCUMFERENCE_MM * ((encoderPosition[1]) / ODO_ENCODER_TICKS)))) / ODO_DISTANCE_MM));
            double angle = (float) Math.toDegrees((encoderPosition[1] - encoderPosition[0]) / (ODO_DISTANCE_MM * ENCODER_TICKS_PER_MM));
            angle=angle+rotOffset;
            pos.setRotation(angle);
            if (Math.abs(botRotDelta) > 0) {
                double radiusOfMovement = (encoderDeltamm[0] + encoderDeltamm[1]) / (2 * botRotDelta); //Radius that robot moves around
                double radiusOfStraif = relativeX / botRotDelta; //radius of the robot while also moving forward :O

                relativeY = (radiusOfMovement * Math.sin(botRotDelta)) - (radiusOfStraif * (1 - Math.cos(botRotDelta)));

                relativeX = radiusOfMovement * (1 - Math.cos(botRotDelta)) + (radiusOfStraif * Math.sin(botRotDelta));
                addPublicTelemetry("relativex ", ""+relativeX);
            }
            pos.translateLocal(relativeY, relativeX, 0);
            pos2.setLocation(pos.getLocation(2), pos.getLocation(1), pos.getLocation(0), pos.getLocation(3));
        } catch (NullPointerException e) {

        }
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
    public Location getPos() {
        try {return pos;}
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
    public String currentRobotPositionString() {return pos.getLocation(0) + ", " + pos.getLocation(2) + ", " + pos.getLocation(3);}

    /* *************************** GETTER METHODS *************************** */

    /**
     * Updates every cycle
     */
    @Override
    public void update(Gamepad gp1, Gamepad gp2) {

    }

    /**
     * Updates every cylcle
     */
    @Override
    public void write() {

    }
}
