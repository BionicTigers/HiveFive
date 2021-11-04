package org.firstinspires.ftc.teamcode;

/**
 * Determines the location of the robot on the field
 * @author Jack 2
 */
public class Location {
    //Fields
    /**Position of the robot*/
    private float[] pos;

    /**
     * Location Constructor Method
     */
    public Location() {
        pos = new float[4];
        setLocation(0, 0, 0, 0);
    }

    /**
     * Location Constructor Method
     * @param location  location of the robot
     */
    public Location(float[] location) {
        pos = new float[4];
        setLocation(location);
    }

    /**
     * Location Constructor Method
     * @param x x position coordinate
     * @param y y position coordinate
     * @param z z position coordinate
     * @param rot rotation
     */
    public Location(float x, float y, float z, float rot) {
        pos = new float[4];
        setLocation(x, y, z, rot);
    }

    /**
     * Location Constructor Method
     * @param x X position coordinate
     * @param y Y position coordinate
     * @param z Z position coordinate
     * @param rot   degrees rotation
     */
    public Location(double x, double y, double z, double rot) {
        pos = new float[4];
        setLocation(x, y, z, rot);
    }

    /* *************************** GETTER METHODS *************************** */

    /**
     * Returns float array of current position. [x,y,z,rotation]
     * @return float[]
     */
    public float[] getLocation() {
        return pos;
    }

    /**
     * Returns float of desired index of position. [x,y,z,rotation]
     * @param index index of the desired return value.
     * @return float
     */
    public float getLocation(int index) {
        if (index < 0 || index > 4)
            throw new IllegalArgumentException("getLocation requires range of 1-4.");
        return pos[index];
    }

    /* *************************** SETTER METHODS *************************** */

    /**
     * Sets location to input.
     * @param location Float array of length 4. [x,y,z,rotation in degrees].
     */
    public void setLocation(float[] location) {
        if (location.length == 4) {
            pos = location;
            pos[3] %= 360;
        } else throw new IllegalArgumentException("Invalid location array: x,y,z,rot required.");
    }

    /**
     * Sets location to input.
     *
     * @param x   X coordinate.
     * @param y   Y coordinate.
     * @param z   Z coordinate.
     * @param rot Rotation in degrees.
     */
    public void setLocation(float x, float y, float z, float rot) {
        pos[0] = x;
        pos[1] = y;
        pos[2] = z;
        pos[3] = rot % 360;
    }

    /**
     * Sets location to input.
     * @param x   X coordinate.
     * @param y   Y coordinate.
     * @param z   Z coordinate.
     * @param rot Rotation in degrees.
     */
    public void setLocation(double x, double y, double z, double rot) {
        pos[0] = (float) x;
        pos[1] = (float) y;
        pos[2] = (float) z;
        pos[3] = (float)(rot % 360);
    }

    /**
     * Sets location coordinate to whatever is input.
     * @param co coordinate of the thing from 0 to 3
     * @param x wanted value
     */
    public void setLocation(int co, float x) {
        pos[co] = x;
    }

    /**
     * Sets stored rotation.
     * New function to correct for negative angle
     * @param rot Rotation in degrees.
     */
    public void setRotation(float rot) {
        double angle = (float) rot % 360;
        if (angle<0){
            angle = 360f+angle;
        }
        pos[3] = (float) angle;
    }
    /**
     * Sets stored rotation.
     * New function to correct for negative angle
     * @param rot Rotation in degrees.
     */
    public void setRotation(double rot) {
        double angle = (float) rot % 360;
        if (angle<0){
            angle = 360f+angle;
        }
        pos[3] = (float) angle;
    }

    /* *************************** TRANSLATE METHODS *************************** */

    /**
     * Translates stored location forward given units based on object rotation.
     * Use negative values for opposite direction.
     *
     * @param forward Translates position given units forward.
     */
    public void translateLocal(float forward) {
        translateLocal(forward, 0f,0f);
    }

    /**
     * Translates stored location forward and right given units based on object rotation.
     * Use negative values for opposite direction.
     *
     * @param forward Translates position given units forward.
     * @param right   Translates position given units right.
     * @param rotRad translates rotation in given units Radian
     */
    public void translateLocal(double forward, double right, double rotRad) {
        this.setRotation((float) Math.toDegrees(rotRad) + this.getLocation(3));
        translateLocal((float) forward, 0f, (float) right);
    }

    /**
     * Translates stored location forward, up, and right given units based on object rotation.
     * Use negative values for opposite direction.
     *
     * @param forward Translates position given units forward.
     * @param right   Translates position given units right.
     * @param up      Translates position given units up.
     */
    public void translateLocal(float forward, float up, float right) {
        pos[0] += (float) (forward * Math.cos(Math.toRadians(pos[3])) + right * Math.cos(Math.toRadians(pos[3] - 90)));
        pos[1] += up;
        pos[2] += (float) (forward * Math.sin(Math.toRadians(pos[3])) + right * Math.sin(Math.toRadians(pos[3] - 90)));
    }

    /**
     * Translates object x and z in world coordinates.
     * Use negative values for opposite direction.
     * @param x Translates position given distance on x axis.
     * @param z Translates position given distance on z axis.
     */
    public void translateWorld(float x, float z) {
        pos[0] += x;
        pos[1] += z;
    }

    /* *************************** TELEMETRY STRING METHODS *************************** */

    /**
     * Returns string representation of location.
     *
     * @return String
     */
    public String toString() {
        return "[" + Math.round(pos[0]*1000)/1000f + ","+ Math.round(1000*pos[2] )/1000f+ "," + Math.round(1000*pos[3])/1000 + "]";
    }
}