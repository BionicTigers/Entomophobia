package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.openftc.revextensions2.RevBulkData;

public class NewOdo extends Mechanism{


    public RevBulkData bulkData;
    /*
    Bulk read assignments
    0 = Left
    1 = Right
    2 = Middle
    */

    //Declares constants that relate to odometry wheels
    //Measurements are all in millimeters

    //Diameter of the odometry wheels
    private static final double odo_diameter = 35.28670491;
    //Gear ratio of the odometry wheels
    private static final double gear_ratio = 2.5;
    //Effective diameter of the odometry wheels based on the gear ratio
    private static final double effective_diameter = odo_diameter * gear_ratio;
    //Number of ticks on the encoders
    private static final double encoder_ticks = 8192;
    //Distance between left odometry module and the center of the robot
    private static final double left_offset = 265.7401;
    //Distance between right odometry module and the center of the robot
    private static final double right_offset = 265.7401;
    //Distance between back odometry module and the center of the robot
    private static final double back_offset = 15.5*25.4;


    public double wheel_circumference = effective_diameter * Math.PI;


    //Previous distance that left odometry module has rotated
    public double preLeftPosition = 0;
    //Previous distance that right odometry module has rotated
    public double preRightPosition = 0;
    //Previous distance that back odometry module has rotated
    public double preBackPosition = 0;
    //Change in left odometry module spin distance since last update
    public double deltaLeftMM = 0;
    //Change in right odometry module spin distance since last update
    public double deltaRightMM = 0;
    //Change in back odometry module spin distance since last update
    public double deltabackMM = 0;

    public double deltaLeftTicks = 0;


    /*Declares an array of encoder values*/
    public double[] encoderDeltamm = new double[3];

    //Distance between robot's center and center of arc rotation
    private double rT = 0;


    public void updateLocalPosition() {
        rT = (deltaLeftMM+deltaRightMM)/(deltaLeftMM-deltaRightMM);

        //Converts change in ticks from last step into change in MM from last step
        deltaLeftMM = wheel_circumference * (bulkData.getMotorCurrentPosition(i) - deltaLeftTicks - (preLeftPosition) / encoder_ticks);
    }


    @Override
    public void update(Gamepad gp1, Gamepad gp2) {

    }

    @Override
    public void write() {

    }
}
