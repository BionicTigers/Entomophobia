package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.teamcode.teleop.NOdoRobot;
import org.firstinspires.ftc.teamcode.util.NewLocation;
import org.firstinspires.ftc.teamcode.util.Variables;

import java.util.ArrayList;
import java.util.HashMap;

public class NewDrivetrain extends Mechanism {
    private Telemetry telemetry;
    private NOdoRobot robot;
    private HashMap<String, Servo> odoServos;
    private ArrayList<DcMotorEx> driveMotors;

    //PID??
    public double[] integralValues;
    public double lastForwardError;
    public double lastSidewaysError;
    public double lastRotationError;

    private double[] motorPowers;

    public NewDrivetrain(NOdoRobot robot, Telemetry telemetry, HashMap<String, Servo> odoServos) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.odoServos = odoServos;

        integralValues = new double[3];

        driveMotors = robot.motors;
        motorPowers = new double[4];

        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Used to determine what powers
     *
     * @param gp1 The Main Gamepad
     */
    public void determineMotorPowers(Gamepad gp1) {
        double power = Math.hypot(-gp1.left_stick_x, -gp1.left_stick_y);
        double robotAngle = Math.atan2(-gp1.left_stick_y, -gp1.left_stick_x);
        double stickAngle = gp1.right_stick_x;

        double angleSin = Math.sin(robotAngle);
        double angleCos = Math.cos(robotAngle);

        double frontMultiplier = gp1.right_bumper ? 0.3 : 1;
        double backMultiplier = gp1.right_bumper ? 0.3 : -1;

        motorPowers[0] = ((power * angleSin) + (power * angleCos) - stickAngle) * frontMultiplier;  //frontRight
        motorPowers[1] = ((power * angleSin) - (power * angleCos) + stickAngle) * frontMultiplier;  //frontLeft
        motorPowers[2] = ((-power * angleSin) - (power * angleCos) - stickAngle) * backMultiplier; //backLeft
        motorPowers[3] = ((-power * angleSin) + (power * angleCos) + stickAngle) * backMultiplier; //backRight
    }

    /*public void determineMotorPowers(double rot, double speed) {
        double power = Math.hypot(-gp1.left_stick_x, -gp1.left_stick_y);
        double robotAngle = Math.atan2(-gp1.left_stick_y, -gp1.left_stick_x);
        double stickAngle = gp1.right_stick_x;

        double angleSin = Math.sin(robotAngle);
        double angleCos = Math.cos(robotAngle);

        double frontMultiplier = gp1.right_bumper ? 0.3 : 1;
        double backMultiplier = gp1.right_bumper ? 0.3 : -1;

        motorPowers[0] = ((power * angleSin) + (power * angleCos) - stickAngle) * frontMultiplier;  //frontRight
        motorPowers[1] = ((power * angleSin) - (power * angleCos) + stickAngle) * frontMultiplier;  //frontLeft
        motorPowers[2] = ((-power * angleSin) - (power * angleCos) - stickAngle) * backMultiplier; //backLeft
        motorPowers[3] = ((-power * angleSin) + (power * angleCos) + stickAngle) * backMultiplier; //backRight
    }*/

    public void determineMotorPowers(double x, double y, double rot) {
        double power = Math.hypot(-x, y);
        double robotAngle = Math.atan2(y, -x);

        double angleSin = Math.sin(robotAngle);
        double angleCos = Math.cos(robotAngle);

        double frontMultiplier = 1;
        double backMultiplier = -1;

        motorPowers[0] = ((power * angleSin) + (power * angleCos) - rot) * frontMultiplier;  //frontRight
        motorPowers[1] = ((power * angleSin) - (power * angleCos) + rot) * frontMultiplier;  //frontLeft
        motorPowers[2] = ((-power * angleSin) - (power * angleCos) - rot) * backMultiplier; //backLeft
        motorPowers[3] = ((-power * angleSin) + (power * angleCos) + rot) * backMultiplier; //backRight
    }

    /** Strafe in a given rotation for a set amount of time
     *
     * @param rot Number from 0 to 1
     */
    public void RotationStrafe(double rot) {
        double scaled = rot*2*Math.PI;

        double power = Math.hypot(-scaled, scaled);
        double robotAngle = Math.atan2(scaled, -scaled);

        double angleSin = Math.sin(robotAngle);
        double angleCos = Math.cos(robotAngle);

        double frontMultiplier = 1;
        double backMultiplier = -1;

        motorPowers[0] = ((power * angleSin) + (power * angleCos) - rot) * frontMultiplier;  //frontRight
        motorPowers[1] = ((power * angleSin) - (power * angleCos) + rot) * frontMultiplier;  //frontLeft
        motorPowers[2] = ((-power * angleSin) - (power * angleCos) - rot) * backMultiplier; //backLeft
        motorPowers[3] = ((-power * angleSin) + (power * angleCos) + rot) * backMultiplier; //backRight
    }

    /** Strafe in a given rotation for a set amount of time
     *
     * @param rot Number from 0 to 1
     * @param time The time that it will strafe (in MS)
     */
    public void RotationStrafe(double rot, double time) {
        double scaled = rot*2*Math.PI;

        double power = Math.hypot(-scaled, scaled);
        double robotAngle = Math.atan2(scaled, -scaled);

        double angleSin = Math.sin(robotAngle);
        double angleCos = Math.cos(robotAngle);

        double frontMultiplier = 1;
        double backMultiplier = -1;

        double startTime = robot.getTimeMS();

        motorPowers[0] = ((power * angleSin) + (power * angleCos) - rot) * frontMultiplier;  //frontRight
        motorPowers[1] = ((power * angleSin) - (power * angleCos) + rot) * frontMultiplier;  //frontLeft
        motorPowers[2] = ((-power * angleSin) - (power * angleCos) - rot) * backMultiplier; //backLeft
        motorPowers[3] = ((-power * angleSin) + (power * angleCos) + rot) * backMultiplier; //backRight

        while (robot.getTimeMS() - startTime < time) {}

        stopDrivetrain();
    }


    /** Strafe in a given rotation for a set amount of time
     *
     * @param rot Number from 0 to 1
     * @param speed Number from 0 to 1
     */
    public void RotationStrafeMod(double rot, double speed) {
        double scaled = rot*2*Math.PI;

        double power = Math.hypot(-scaled, scaled);
        double robotAngle = Math.atan2(scaled, -scaled);

        double angleSin = Math.sin(robotAngle);
        double angleCos = Math.cos(robotAngle);

        double inverseSpeed = -speed;

        motorPowers[0] = ((power * angleSin) + (power * angleCos) - rot) * speed;  //frontRight
        motorPowers[1] = ((power * angleSin) - (power * angleCos) + rot) * speed;  //frontLeft
        motorPowers[2] = ((-power * angleSin) - (power * angleCos) - rot) * inverseSpeed; //backLeft
        motorPowers[3] = ((-power * angleSin) + (power * angleCos) + rot) * inverseSpeed; //backRight
    }

    /** Strafe in a given rotation for a set amount of time
     *
     * @param rot Number from 0 to 1
     * @param time The time that it will strafe (in MS)
     * @param speed Number from 0 to 1
     */
    public void RotationStrafeMod(double rot, double time, double speed) {
        double scaled = rot*2*Math.PI;

        double power = Math.hypot(-scaled, scaled);
        double robotAngle = Math.atan2(scaled, -scaled);
        double stickAngle = rot;

        double angleSin = Math.sin(robotAngle);
        double angleCos = Math.cos(robotAngle);

        double frontMultiplier = speed;
        double backMultiplier = -speed;

        double startTime = robot.getTimeMS();

        motorPowers[0] = ((power * angleSin) + (power * angleCos) - stickAngle) * frontMultiplier;  //frontRight
        motorPowers[1] = ((power * angleSin) - (power * angleCos) + stickAngle) * frontMultiplier;  //frontLeft
        motorPowers[2] = ((-power * angleSin) - (power * angleCos) - stickAngle) * backMultiplier; //backLeft
        motorPowers[3] = ((-power * angleSin) + (power * angleCos) + stickAngle) * backMultiplier; //backRight

        while (robot.getTimeMS() - startTime < time) {}

        stopDrivetrain();
    }

    public void stopDrivetrain() {
        determineMotorPowers(0,0,0);
        this.write();
    }

    public void moveToPosition(NewLocation goalPos, double xTolerance, double zTolerance, double rotTolerance, int maxTime) {
        NewLocation error = findError(goalPos);
        double startTime = robot.getTimeMS();
        while ((robot.getTimeMS() - startTime < maxTime) && robot.linoop.opModeIsActive()&&(Math.abs(error.x) > xTolerance || Math.abs(error.y) > zTolerance || Math.abs(error.rot) > rotTolerance)) {
            error = findError(goalPos);
            write();
            robot.odometry.updateLocalPosition();
            robot.odometry.updateGlobalPosition();
        }
        stopDrivetrain();
    }

    public void moveToPosition(NewLocation goalPos, double xTolerance, double zTolerance, double rotTolerance) {
        NewLocation error = findError(goalPos);
        while (robot.linoop.opModeIsActive()&&(Math.abs(error.x) > xTolerance || Math.abs(error.y) > zTolerance || Math.abs(error.rot) > rotTolerance)) {
            error = findError(goalPos);
            write();
            robot.odometry.updateLocalPosition();
            robot.odometry.updateGlobalPosition();
        }
        stopDrivetrain();
    }

    public void moveToPositionMod(NewLocation goalPos, double xTolerance, double zTolerance, double rotTolerance, int maxTime, double mod) {
        NewLocation error = findError(goalPos);
        double startTime = robot.getTimeMS();
        while ((robot.getTimeMS() - startTime < maxTime) && robot.linoop.opModeIsActive()&&(Math.abs(error.x) > xTolerance || Math.abs(error.y) > zTolerance || Math.abs(error.rot) > rotTolerance)) {
            error = findError(goalPos);
            write();
            robot.odometry.updateLocalPosition();
            robot.odometry.updateGlobalPosition();
        }
        stopDrivetrain();
    }

    public void moveToPositionMod(NewLocation goalPos, double xTolerance, double zTolerance, double rotTolerance, double mod) {
        NewLocation error = findError(goalPos, mod);
        while (robot.linoop.opModeIsActive()&&(Math.abs(error.x) > xTolerance || Math.abs(error.y) > zTolerance || Math.abs(error.rot) > rotTolerance)) {
            error = findError(goalPos, mod);
            write();
            robot.odometry.updateLocalPosition();
            robot.odometry.updateGlobalPosition();
        }
        stopDrivetrain();
    }

    public NewLocation findError(NewLocation goal) {
        //TODO: Change robot .getLocation() to new .x, .y, .rot
        //Subtracts one location from another location
        //I was told not to make a method for this (Blame ben and jamie)
        NewLocation error = new NewLocation(
                goal.x - robot.odometry.position.getLocation(0),
                goal.y - robot.odometry.position.getLocation(1),
                rotationError(goal.rot, robot.odometry.position.getLocation(2))
        );

        double magnitude = Math.hypot(-error.x, error.y);
        double robotheading = Math.atan2(error.x, error.y);

        double forwardError = Math.cos(robotheading-Math.toRadians(robot.odometry.position.getLocation(2)))*magnitude;
        double strafeError = Math.sin(robotheading-Math.toRadians(robot.odometry.position.getLocation(2)))*magnitude;

        if(Math.abs(Variables.kfP*forwardError + Variables.kfI*integralValues[0] + Variables.kfD * (forwardError- lastForwardError))<1)
            integralValues[0] = integralValues[0]+forwardError;
        if(Math.abs(Variables.ksP*strafeError + Variables.ksI*integralValues[2] + Variables.ksD * (strafeError - lastSidewaysError))<1)
            integralValues[1] = integralValues[1]+strafeError;
        if(Math.abs(Variables.krP*error.rot + Variables.krI*integralValues[3] + Variables.krD * (error.rot - lastRotationError))<1)
            integralValues[2] = integralValues[2]+error.rot;

        double forwardPow = (Variables.kfP*forwardError+ Variables.kfI*integralValues[0] + Variables.kfD * (forwardError - lastForwardError));
        double sidePow = (Variables.ksP*strafeError + Variables.ksI*integralValues[2] + Variables.ksD * (strafeError - lastSidewaysError)) ;
        double rotPow = -(Variables.krP *error.rot + Variables.krI*integralValues[3] +Variables.krD * ( error.rot - lastRotationError));

        lastForwardError = forwardPow;
        lastSidewaysError = sidePow;
        lastRotationError = rotPow;

        determineMotorPowers(sidePow,forwardPow,rotPow);
        return error;
    }

    public NewLocation findError(NewLocation goal, double mod) {
        //TODO: Change robot .getLocation() to new .x, .y, .rot
        //Subtracts one location from another location
        //I was told not to make a method for this (Blame ben and jamie)
        NewLocation error = new NewLocation(
                goal.x - robot.odometry.position.getLocation(0),
                goal.y - robot.odometry.position.getLocation(1),
                rotationError(goal.rot, robot.odometry.position.getLocation(2))
        );

        double magnitude = Math.hypot(-error.x, error.y);
        double robotheading = Math.atan2(error.x, error.y);

        double forwardError = Math.cos(robotheading-Math.toRadians(robot.odometry.position.getLocation(2)))*magnitude;
        double strafeError = Math.sin(robotheading-Math.toRadians(robot.odometry.position.getLocation(2)))*magnitude;

        if(Math.abs(Variables.kfP*forwardError + Variables.kfI*integralValues[0] + Variables.kfD * (forwardError- lastForwardError))<1)
            integralValues[0] = integralValues[0]+forwardError;
        if(Math.abs(Variables.ksP*strafeError + Variables.ksI*integralValues[2] + Variables.ksD * (strafeError - lastSidewaysError))<1)
            integralValues[1] = integralValues[1]+strafeError;
        if(Math.abs(Variables.krP*error.rot + Variables.krI*integralValues[3] + Variables.krD * (error.rot - lastRotationError))<1)
            integralValues[2] = integralValues[2]+error.rot;

        double forwardPow = mod*(Variables.kfP*forwardError+ Variables.kfI*integralValues[0] + Variables.kfD * (forwardError - lastForwardError));
        double sidePow = mod*(Variables.ksP*strafeError + Variables.ksI*integralValues[2] + Variables.ksD * (strafeError - lastSidewaysError)) ;
        double rotPow = -(Variables.krP *error.rot + Variables.krI*integralValues[3] +Variables.krD * ( error.rot - lastRotationError));

        lastForwardError = forwardPow;
        lastSidewaysError = sidePow;
        lastRotationError = rotPow;

        determineMotorPowers(sidePow,forwardPow,rotPow);
        return error;
    }

    //Subtract rot2 (current) from rot1 (goal) while making it between -180 and 180
    public double rotationError(double rot1, double rot2) {
        double spinError = rot1 - rot2;

        //This is basically just a condensed if statement
        //If spinError > 180 then spinError -= 360
        spinError = spinError > 180 ? spinError - 360 : spinError;
        spinError = spinError < -180 ? spinError + 360 : spinError;
        spinError = spinError == 180 ? 20 : spinError;

        return spinError;
    }

    public void odoUp () {
        odoServos.get("Left").setPosition(0.45);
        odoServos.get("Middle").setPosition(0.3);
        odoServos.get("Right").setPosition(0.45);
    }

    public void odoDown () {
        odoServos.get("Left").setPosition(0.17);
        odoServos.get("Middle").setPosition(0.64);
        odoServos.get("Right").setPosition(1);
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp1.y) {
            odoUp();
        } else if (gp1.b) {
            odoDown();
        }

        determineMotorPowers(gp1);
    }

    public void write() {
        for (int i = 0; i < driveMotors.size(); i++) {
            driveMotors.get(i).setPower(motorPowers[i]);
        }
    }
}
