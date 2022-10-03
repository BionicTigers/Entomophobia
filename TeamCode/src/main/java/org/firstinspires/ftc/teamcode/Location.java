
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/*
 * A custom class to store the robot's position data. Stores X, Y, Z, and Azimuth (degrees).
 */
public class Location {
    public float[] position;

    public Location() {
        position = new float[4];
        setLocation(0, 0, 0, 0);
    }
    public Location(float[] location) {
        position = new float[4];
        setLocation(location);
    }

    public Location(float x, float y, float z, float rot) {
        position = new float[4];
        setLocation(x, y, z, rot);
    }
    public Location(double x, double y, double z, double rot) {
        position = new float[4];
        setLocation(x, y, z, rot);
    }
    /*
     * Returns float array of current position. [x,y,z,rotation]
     *
     * @return float[]
     */
    public float[] getLocation() {
        return position;
    }

    /*
     * Returns float of desired index of position. [x,y,z,rotation]
     *
     * @param index index of the desired return value.
     * @return float
     */
    public float getLocation(int index) {
        if (index < 0 || index > 4)
            throw new IllegalArgumentException("getLocation requires range of 1-4.");
        return position[index];
    }

    /*
     * Sets location to input.
     *
     * @param location Float array of length 4. [x,y,z,rotation in degrees].
     */
    public void setLocation(float[] location) {
        if (location.length == 4) {
            position = location;
            position[3] %= 360;
        } else throw new IllegalArgumentException("Invalid location array: x,y,z,rot required.");
    }

    /*
     * Sets location to input.
     *
     * @param x   X coordinate.
     * @param y   Y coordinate.
     * @param z   Z coordinate.
     * @param rot Rotation in degrees.
     */
    public void setLocation(float x, float y, float z, float rot) {
        position[0] = x;
        position[1] = y;
        position[2] = z;
        position[3] = rot % 360;
    }
    public void setLocation(double x, double y, double z, double rot) {
        position[0] = (float)x;
        position[1] = (float)y;
        position[2] = (float)z;
        position[3] = (float)(rot % 360);
    }

    /*
     * Sets location coordinate to whatever is input.
     *
     * @param co coordinate of the thing from 0 to 3
     * @param x wanted value
     */
    public void setLocation(int co, float x) {
        position[co] = x;
    }

    /*
     * Sets location to input.
     *
     * @param o OpenGLMatrix for position.
     */
    public void setLocation(OpenGLMatrix o) {
        position[0] = o.getTranslation().get(0);
        position[1] = o.getTranslation().get(1);
        position[2] = o.getTranslation().get(2);
        position[3] = o.getTranslation().get(3); //<- test plz
    }

    /*
     * Sets stored rotation.
     *     *New function to correct for negative angle
     * @param rot Rotation in degrees.
     */
    public void setRotation(float rot) {
        double angle = (float) rot % 360;
        if (angle<0){
            angle = 360f+angle;
        }
        position[3] = (float) angle;
    }
    /*
     * Sets stored rotation.
     *New function to correct for negative angle
     * @param rot Rotation in degrees.
     */
    public void setRotation(double rot) {
        double angle = (float) rot % 360;
        if (angle<0){
            angle = 360f+angle;
        }
        position[3] = (float) angle;
    }


    /*
     * Translates stored location forward given units based on object rotation.
     * Use negative values for opposite direction.
     *
     * @param forward Translates position given units forward.
     */
    public void translateLocal(float forward) {
        translateLocal(forward, 0f,0f);
    }

    /*
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

    /*
     * Translates stored location forward, up, and right given units based on object rotation.
     * Use negative values for opposite direction.
     *
     * @param forward Translates position given units forward.
     * @param right   Translates position given units right.
     * @param up      Translates position given units up.
     */
    public void translateLocal(float forward, float up, float right) {
        position[0] += (float) (forward * Math.cos(Math.toRadians(position[3])) + right * Math.cos(Math.toRadians(position[3] - 90)));
        position[1] += up;
        position[2] += (float) (forward * Math.sin(Math.toRadians(position[3])) + right * Math.sin(Math.toRadians(position[3] - 90)));
    }
    public void openGLTranslateLocal(float forward, float right, float rot){

        setLocation(new OpenGLMatrix(new float[]{right,forward,0,(float ) Math.toDegrees(rot)}));
    }

    /*
     * Translates object x and z in world coordinates.
     * Use negative values for opposite direction.
     * @param x Translates position given distance on x axis.
     * @param z Translates position given distance on z axis.
     */
    public void translateWorld(float x, float z) {
        position[0] += x;
        position[1] += z;
    }

    /*
     * Returns string representation of location.
     *
     * @return String
     */
    public String toString() {
        return "[" + Math.round(position[0]*1000)/1000f + ","+ Math.round(1000* position[2] )/1000f+ "," + Math.round(1000* position[3])/1000 + "]";
    }

}