package org.firstinspires.ftc.teamcode.util;

//A custom class to store the robot's position data. Stores X, Y, and Azimuth (degrees).
public class Location {
    public double[] position;

    public Location() {
        position = new double[3];
        setLocation(0, 0, 0);
    }

    public Location(double x, double y, double rot) {
        position = new double[3];
        setLocation(x, y, rot);
    }

    /**
     * Returns float array of current position. [x,y,rotation]
     *
     * @return float[]
     */
    public double[] getLocation() {
        return position;
    }

    /**
     * Returns float of desired index of position. [x,y,rotation]
     *
     * @param index index of the desired return value.
     * @return float
     */
    public double getLocation(int index) {
        if (index < 0 || index > 2)
            throw new IllegalArgumentException("getLocation requires range of 0-3.");
        return position[index];
    }

    /**
     * Sets location to input.
     *
     * @param location Float array of length 3. [x,y,rotation in degrees].
     */
    public void setLocation(double[] location) {
        if (location.length == 3) {
            position = location;
            position[2] %= 360;
        } else throw new IllegalArgumentException("Invalid location array: x,y,rot required.");
    }

    public void setLocation(double x, double y, double rot) {
        position[0] = x;
        position[1] = y;
        position[2] = (rot % 360);
    }

    /**
     * Sets location coordinate to whatever is input.
     *
     * @param co coordinate of the thing from 0 to 3
     * @param x  wanted value
     */
    public void setLocation(int co, double x) {
        position[co] = x;
    }

    /**
     * Sets stored rotation.
     * New function to correct for negative angle
     *
     * @param rot Rotation in degrees.
     */
    public void setRotation(double rot) {
        double angle = rot % 360;
        if (angle < 0) {
            angle = 360 + angle;
        }
        position[2] = angle;
    }

    /**
     * Returns string representation of location.
     *
     * @return String
     */
    public String toString() {
        return "[" + Math.round(position[0] * 1000) / 1000 + "," + Math.round(1000 * position[2]) / 1000 + "," + Math.round(1000 * position[3]) / 1000 + "]";
    }
}