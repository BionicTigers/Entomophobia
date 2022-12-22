package org.firstinspires.ftc.teamcode;

//A custom class to store the robot's position data. Stores X, Y, and Azimuth (degrees).
public class NOdoLocation {
    public double x;
    public double y;
    public double rot;

    public NOdoLocation() {
        this.x = 0;
        this.y = 0;
        this.rot = 0;
    }

    public NOdoLocation(double x, double y, double rot) {
        this.x = x;
        this.y = y;
        this.rot = rot;
    }

    /**
     * Sets location to input.
     * @param location Float array of length 3. [x,y,rotation in degrees].
     */
    public void setLocation(double[] location) {
        if (location.length == 3) {
            this.x = location[0];
            this.y = location[1];
            this.rot = location[2] % 360;
        } else throw new IllegalArgumentException("Invalid location array: x,y,rot required.");
    }

    public void setLocation(double x, double y, double rot) {
        this.x = x;
        this.y = y;
        this.rot = rot % 360;
    }

    /**
     * Sets stored rotation.
     *New function to correct for negative angle
     * @param rot Rotation in degrees.
     */
    public void setRotation(double rot) {
        double angle = rot % 360;
        if (angle < 0) {
            angle = 360 + angle;
        }
        this.rot = angle;
    }

    /**
     * Returns string representation of location.
     * @return String
     */
    public String toString() {
        return "[" + Math.round(this.x*1000)/1000 + ","+ Math.round(1000*this.y)/1000 + "," + Math.round(1000*this.rot)/1000 + "]";
    }

}