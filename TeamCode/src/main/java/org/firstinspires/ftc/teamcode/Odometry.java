package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AutoStuff.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.AutoStuff.RevBulkData;
import org.firstinspires.ftc.teamcode.Odometry2;

public class Odometry extends Mechanism{


    /**
     * Constants that have to do with the dimensions of the odometry wheels. Don't make them public please!
     */
    private static double ODO_DIAMETER_MM = 50.8;
    private static double ODO_ENCODER_TICKS = 8192;
    private static double ODO_DISTANCE_MM = 375.7;
    private static double ODO_CIRCUMFERENCE_MM = ODO_DIAMETER_MM * Math.PI;
    private static double ODO_DISTANCE_FROM_CENTER = 38.1; //38.1, 19.05
    private static double ENCODER_TICKS_PER_MM = ODO_ENCODER_TICKS / ODO_CIRCUMFERENCE_MM;

    //Current position fields
    private Location position = new Location();
    public Location realMaybe = new Location();
    public double relativeX;
    public double relativeY;
    private float rotOffset=0;
    private int[] encoderPosition = new int[3];
    private int[] encoderPositionoffset = new int[3];

    public double[] encoderDeltamm = new double[3];

    //Expansion hub data we need for the encoders
    private final ExpansionHubEx expansionHub;
    //private final ExpansionHubEx expansionHub2;
    private RevBulkData bulkData;


    public Mechanism mechy;


    /* *************************** CONSTRUCTOR METHODS *************************** */
    public Odometry(HardwareMap hardwareMap) {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        reset();

    }
    public Odometry(HardwareMap hardwareMap, Location startPos) {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        reset(startPos);
    }

    //New odometry constructor for new robot! The distance and distance from center will be different
    public Odometry(HardwareMap hardwareMap, double distance, double centerDistance,Location startingLocation) {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        reset(startingLocation);
        ODO_DISTANCE_MM = 414.25;
        ODO_DISTANCE_FROM_CENTER = -56.92; //-88.42
        ODO_CIRCUMFERENCE_MM = Math.PI * ODO_DIAMETER_MM;
        ENCODER_TICKS_PER_MM = ODO_ENCODER_TICKS / ODO_CIRCUMFERENCE_MM;
    }
    //New odometry constructor for new robot! The distance and distance from center will be different
    public Odometry(HardwareMap hardwareMap, double distance, double centerDistance) {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        reset();
        ODO_DISTANCE_MM = 420.478;
        ODO_DISTANCE_FROM_CENTER = -56.92; //-88.42
        ODO_CIRCUMFERENCE_MM = Math.PI * ODO_DIAMETER_MM;
        ENCODER_TICKS_PER_MM = ODO_ENCODER_TICKS / ODO_CIRCUMFERENCE_MM;
    }


    /* *************************** ALL THEM OTHER METHODS *************************** */

    /* Reset methods reset the robot's position, either to whatever argument is/arguments are passed
     * in, or, in the case of no arguments, to (0, 0, 0).
     * In addition to resetting the location, it also resets the encoder positions.
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
        } catch (NullPointerException ResetNoParams) {

        }
    }

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
        } catch (NullPointerException ResetXZRotParams) {

        }
    }

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
            realMaybe=new Location(resetPos.getLocation(2),0,resetPos.getLocation(0),resetPos.getLocation(3));

            rotOffset = resetPos.getLocation(3);
        } catch (NullPointerException ResetLocationParams) {

        }
    }

    /*
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
            mechy.addPublicTelemetry("",""+encoderDeltamm[0]);
            mechy.addPublicTelemetry("",""+encoderDeltamm[1]);
            mechy.addPublicTelemetry("",""+encoderDeltamm[2]);


            double botRotDelta = (encoderDeltamm[0] - encoderDeltamm[1]) / ODO_DISTANCE_MM;  //finds change in robo rotation
            relativeX = encoderDeltamm[2] + (ODO_DISTANCE_FROM_CENTER * botRotDelta); //strafing distance
            relativeY = (encoderDeltamm[0] + encoderDeltamm[1]) / 2; //how much moved forward/back
            /*setting current robo rotation in Location object*/
            // pos.setRotation((float) Math.toDegrees(((ODO_CIRCUMFERENCE_MM/*circumference*/ * ((encoderPosition[0]) / ODO_ENCODER_TICKS)/*percentage of the wheel revolved*/ - (ODO_CIRCUMFERENCE_MM * ((encoderPosition[1]) / ODO_ENCODER_TICKS)))) / ODO_DISTANCE_MM));
            double angle = (float) Math.toDegrees((encoderPosition[1] - encoderPosition[0]) / (ODO_DISTANCE_MM * ENCODER_TICKS_PER_MM));
            angle=angle+rotOffset;
            position.setRotation(angle);
            if (Math.abs(botRotDelta) > 0) {
                double radiusOfMovement = (encoderDeltamm[0] + encoderDeltamm[1]) / (2 * botRotDelta); //Radius that robot moves around
                double radiusOfStraif = relativeX / botRotDelta; //radius of the robot while also moving forward :O

                relativeY = (radiusOfMovement * Math.sin(botRotDelta)) - (radiusOfStraif * (1 - Math.cos(botRotDelta)));

                relativeX = radiusOfMovement * (1 - Math.cos(botRotDelta)) + (radiusOfStraif * Math.sin(botRotDelta));
                mechy.addPublicTelemetry("relativex ", ""+relativeX);
            }
            position.translateLocal(relativeY, relativeX, 0);
            realMaybe.setLocation(position.getLocation(2), position.getLocation(1), position.getLocation(0), position.getLocation(3));
        } catch (NullPointerException UpdatePositionNoParams) {

        }
    }

    /* *************************** GETTER METHODS *************************** */
    public int[] getEncoderPosition() {return encoderPosition;}

    public Location getPosition() {
        try {return position;}
        catch (NullPointerException GetPositionNoParams) {
            return new Location();
        }
    }

    /* SETTER METHODS */
    public static void setOdoDiameterMm(double odoDiameterMm) {
        ODO_DIAMETER_MM = odoDiameterMm;
    }

    public static void setOdoEncoderTicks(double odoEncoderTicks) {
        ODO_ENCODER_TICKS = odoEncoderTicks;
    }

    public static void setOdoDistanceMm(double odoDistanceMm) {
        ODO_DISTANCE_MM = odoDistanceMm;
    }

    public static void setOdoCircumferenceMm(double odoCircumferenceMm) {
        ODO_CIRCUMFERENCE_MM = odoCircumferenceMm;
    }

    public static void setOdoDistanceFromCenter(double odoDistanceFromCenter) {
        ODO_DISTANCE_FROM_CENTER = odoDistanceFromCenter;
    }

    public static void setEncoderTicksPerMm(double encoderTicksPerMm) {
        ENCODER_TICKS_PER_MM = encoderTicksPerMm;
    }


    /* *************************** Some fun String methods for telemetry usage *************************** */
    public String currentEncoderPosString() {return encoderPosition[0] + ", " + encoderPosition[1] + ", " + encoderPosition[2];}
    public String currentRobotPositionString() {return position.getLocation(0) + ", " + position.getLocation(2) + ", " + position.getLocation(3);}

    /* *************************** UPDATE AND WRITE METHODS *************************** */
    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp1.back) {
            reset();
        }
        updatePosition();
    }

    public void write() {
        //print position in telemetry
    }
}