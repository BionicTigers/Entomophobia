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
    public double deltaBackMM = 0;

    public double deltaLeftTicks = 0;
    public double deltaRightTicks = 0;
    public double deltaBackTicks = 0;

    //Distance between robot's center and center of arc rotation
    private double rT = 0;


    public void updateLocalPosition() {
        rT = (deltaLeftMM+deltaRightMM)/(deltaLeftMM-deltaRightMM);

        for (int i = 0; i == 2; i++) {
            if (i == 1) {
                //Converts change in ticks from last step into change in MM from last step for the left wheel
                deltaLeftMM = -wheel_circumference * (bulkData.getMotorCurrentPosition(i) - deltaLeftTicks + (preLeftPosition) / encoder_ticks);
            } else if (i == 2) {
                //Converts change in ticks from last step into change in MM from last step for the right wheel
                deltaRightMM = -wheel_circumference * (bulkData.getMotorCurrentPosition(i) - deltaRightTicks + (preRightPosition) / encoder_ticks);
            } else {
                //Converts change in ticks from last step into change in MM from last step for the back wheel
                deltaBackMM = -wheel_circumference * (bulkData.getMotorCurrentPosition(i) - deltaBackTicks + (preBackPosition) / encoder_ticks);
            }
        }
    }


    @Override
    public void update(Gamepad gp1, Gamepad gp2) {

    }

    @Override
    public void write() {

    }
}
