package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

public class NewOdo extends Mechanism{

    private final ExpansionHubEx expansionHub;
    public Telemetry telemetry;
    //Bulk data
    public RevBulkData bulkData;
    //bulkData.getMotorCurrentPosition(i) gets the total ticks rotated since power up

    /*
    Bulk read assignments
    0 = Left
    1 = Right
    2 = Back
    */

    public NOdoLocation location;

    public NOdoLocation position = new NOdoLocation();

    //Declares constants that relate to odometry wheels
    //Measurements are all in millimeters

    //Diameter of the odometry wheels
    private static final double odo_diameter = 25.4;
    //Gear ratio of the odometry wheels
    private static final double gear_ratio = 2.5;
    //Number of ticks on the encoders
    private static final double encoder_ticks = 8192;
    //Distance between left odometry module and the center of the robot
    private static final double left_offset = 165.5;
    //Distance between right odometry module and the center of the robot
    private static final double right_offset = 165.5;
    //Distance between back odometry module and the center of the robot
    private static final double back_offset = 73;

    //Calculates the effective diameter of the odometry wheels based on the gear ratio
    private static final double effective_diameter = odo_diameter * gear_ratio;
    //Calculates the circumference of the odometry wheel
    public double wheel_circumference = effective_diameter * Math.PI;


    //Distance the odometry wheel was at last cycle in ticks
    public double previousLeftTicks = 0;
    public double previousRightTicks = 0;
    public double previousBackTicks = 0;

    //Change in odometry module spin distance in MM since last cycle
    public double deltaLeftMM = 0;
    public double deltaRightMM = 0;
    public double deltaBackMM = 0;

    //The amount of ticks that are left over from a bulkdata pull before an odometry reset
    public double junkLeftTicks = 0;
    public double junkRightTicks = 0;
    public double junkBackTicks = 0;

    //The amount of ticks rotated after an odometry reset
    public double postResetLeftTicks = 0;
    public double postResetRightTicks = 0;
    public double postResetBackTicks = 0;

    //Change in rotation in radians since last cycle
    private double deltaLocalRotation = 0;
    //Distance between robot's center and center of arc rotation
    private double rT = 0;
    //Change in local straight line distance since last cycle
    //Currently unused, just needed to calculate for a proof but, this is the code so proofs aren't as important
    private double deltaLocalDistance = 0;
    //Change in local coordinates that the robot moved since last cycle
    private double deltaLocalX = 0;
    private double deltaLocalY = 0;
    //Radius of the strafe to arc center
    private double rS = 0;
    //Change in local coordinates that the robot strafed since last cycle
    private double deltaXStrafe = 0;
    private double deltaYStrafe = 0;
    //Change in local coordinates of strafe and forward arcs since last cycle
    private double deltaXFinal = 0;
    private double deltaYFinal = 0;
    //Change in global straight line distance since last cycle
    private double deltaGlobalDistance = 0;
    //Change in global coordinates since last cycle
    private double deltaGlobalY = 0;
    private double deltaGlobalX = 0;


    //The rotation of the robot relative to it's starting rotation
    private double globalRotation = 0;
    //The global x and y position of the robot relative to it's starting location
    private double globalX = 0;
    private double globalY = 0;

    //Our robot's global rotation in degrees
    public double rotationInDegrees = 0;


    public NewOdo(HardwareMap hardwareMap, Telemetry T) {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        reset();

        telemetry = T;
    }
    public void updateLocalPosition() {
        bulkData = expansionHub.getBulkInputData();

        //Calculates the amount of ticks spun since reset by subtracting the junk from before rest from total rotated
        postResetLeftTicks = bulkData.getMotorCurrentPosition(0) - junkLeftTicks;
        postResetRightTicks = bulkData.getMotorCurrentPosition(1) - junkRightTicks;
        postResetBackTicks = bulkData.getMotorCurrentPosition(2) - junkBackTicks;

        //Converts change in ticks from last cycle into change in MM from last cycle for each wheel, and 1 & 2 are inversed
        deltaLeftMM = wheel_circumference * ((postResetLeftTicks - previousLeftTicks) / encoder_ticks);
        deltaRightMM = -wheel_circumference * ((postResetRightTicks - previousRightTicks) / encoder_ticks);
        deltaBackMM = -wheel_circumference * ((postResetBackTicks - previousBackTicks) / encoder_ticks);

        //Displays telemetry for the change in MM from last cycle for each wheel
        addPublicTelemetry("",""+deltaLeftMM);
        addPublicTelemetry("",""+deltaRightMM);
        addPublicTelemetry("",""+deltaBackMM);

        //Calculates the change in local angle of the robot after the movement
        deltaLocalRotation = (deltaLeftMM-deltaRightMM) / (left_offset + right_offset);
        //Calculates the radius of the arc of the robot's travel for forward/backward arcs
        //If statement ensures that if deltaLeftMM - deltaRightMM equals 0, our rT won't return as null
        if (deltaRightMM != deltaLeftMM) {
            rT = (deltaLeftMM + deltaRightMM) / (deltaLeftMM - deltaRightMM);
            //Determine the local x and y coordinates for a forward/backward arc
            deltaLocalX = rT * (1 - Math.cos(deltaLocalRotation));
            deltaLocalY = rT * Math.sin(deltaLocalRotation);
        } else {
            deltaLocalX = deltaLeftMM;
            deltaLocalY = 0;
        }

        //Calculates the straight-line distance between the starting and ending points of the robot's local travel
        //Currently unused, just needed to calculate for a proof but, this is the code so proofs aren't as important
        deltaLocalDistance = 2 * rT * Math.sin(deltaLocalRotation / 2);

        //Calculates the radius of a strafing arc
        //If statement ensures that if deltaLocalRotation is 0, our rS won't return as null
        if (deltaLocalRotation != 0) {
            rS = (deltaBackMM / deltaLocalRotation) - back_offset;
            //Determine the local x and y coordinates for a strafing arc
            deltaXStrafe = rS * Math.sin(deltaLocalRotation);
            deltaYStrafe = rS * (1 - Math.cos(deltaLocalRotation));
        } else {
            deltaXStrafe = 0;
            deltaYStrafe = deltaBackMM;
        }
        //Calculates the total local x and y changes since last cycle
        deltaXFinal = deltaLocalX + deltaXStrafe;
        deltaYFinal = deltaLocalY - deltaYStrafe;

        //Sets the previous move tick amounts to whatever the ticks were at the end of this last cycle
        previousLeftTicks = postResetLeftTicks;
        previousRightTicks = postResetRightTicks;
        previousBackTicks = postResetBackTicks;
    }


    public void updateGlobalPosition() {
        //Change in straight line distance of move on the global
        deltaGlobalDistance = Math.sqrt((deltaXFinal * deltaXFinal) + (deltaYFinal * deltaYFinal));
        //Updates the global rotation of the robot compared to the starting angle
        globalRotation = globalRotation + deltaLocalRotation;
        //Calculates the change in global x and y coordinates since last cycle
//        deltaGlobalX = deltaGlobalDistance * Math.sin(globalRotation + (deltaLocalRotation / 2));
//        deltaGlobalY = deltaGlobalDistance * Math.cos(globalRotation + (deltaLocalRotation / 2));
//        //Updates the x and y coordinates of the robot compared to the starting position
//        globalX = globalX + deltaGlobalX;
//        globalY = globalY + deltaGlobalY;

        //Testing stuff.. Yipeee!!
        globalX += deltaXFinal * Math.cos(globalRotation) + deltaYFinal * Math.cos(globalRotation - (Math.PI / 2));
        globalY += deltaXFinal * Math.sin(globalRotation) + deltaYFinal * Math.sin(globalRotation - (Math.PI / 2));

        //Converts our global rotation to degrees
        rotationInDegrees = Math.toDegrees(globalRotation);
    }

    public void reset() {
        //Sets x, y, and rotation to 0
        globalRotation = 0;
        globalX = 0;
        globalY = 0;

        bulkData = expansionHub.getBulkInputData();

        //Sets our current position to the zero point
        position.setLocation(0, 0, 0);

        //Sets the leftover junk ticks to the current motor rotations
        junkLeftTicks = bulkData.getMotorCurrentPosition(0);
        junkRightTicks = bulkData.getMotorCurrentPosition(1);
        junkBackTicks = bulkData.getMotorCurrentPosition(2);
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        updateLocalPosition();
        updateGlobalPosition();
    }

    @Override
    public void write() {
        telemetry.addData("",")");
        telemetry.addData("Left Tick Rotation", postResetLeftTicks);
        telemetry.addData("Right Tick Rotation", postResetRightTicks);
        telemetry.addData("Back Tick Rotation", postResetBackTicks);
        telemetry.addData("Back Tick Delta", deltaBackMM);
        telemetry.addData("Position", "");
        telemetry.addData("X", globalX);
        telemetry.addData("Y", globalY);
        telemetry.addData("Rot", rotationInDegrees);
        telemetry.addData("DeltaGlobalY", deltaGlobalY);
    }
}