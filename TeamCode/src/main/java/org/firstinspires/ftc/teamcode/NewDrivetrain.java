package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;

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

    public void determineMotorPowers(double x, double y, double rot) {
        double power = Math.hypot(-x, y);
        double robotAngle = Math.atan2(y, -x);
        double stickAngle = rot;

        double angleSin = Math.sin(robotAngle);
        double angleCos = Math.cos(robotAngle);

        double frontMultiplier = 1;
        double backMultiplier = -1;

        motorPowers[0] = ((power * angleSin) + (power * angleCos) - stickAngle) * frontMultiplier;  //frontRight
        motorPowers[1] = ((power * angleSin) - (power * angleCos) + stickAngle) * frontMultiplier;  //frontLeft
        motorPowers[2] = ((-power * angleSin) - (power * angleCos) - stickAngle) * backMultiplier; //backLeft
        motorPowers[3] = ((-power * angleSin) + (power * angleCos) + stickAngle) * backMultiplier; //backRight
    }

    public void stopDrivetrain() {
        determineMotorPowers(0,0,0);
        this.write();
    }

    public void moveToPosition(NOdoLocation goalPos, double xTolerance, double zTolerance, double rotTolerance, int maxTime) {
        NOdoLocation error = findError(goalPos);
        double startTime = robot.getTimeMS();
        while ((robot.getTimeMS() - startTime < maxTime) && robot.linoop.opModeIsActive()&&(Math.abs(error.x) > xTolerance || Math.abs(error.y) > zTolerance || Math.abs(error.rot) > rotTolerance)) {
            error = findError(goalPos);
            write();
            robot.odometry.updateLocalPosition();
            robot.odometry.updateGlobalPosition();
        }
        stopDrivetrain();
    }

    public void moveToPosition(NOdoLocation goalPos, double xTolerance, double zTolerance, double rotTolerance) {
        NOdoLocation error = findError(goalPos);
        while (robot.linoop.opModeIsActive()&&(Math.abs(error.x) > xTolerance || Math.abs(error.y) > zTolerance || Math.abs(error.rot) > rotTolerance)) {
            error = findError(goalPos);
            write();
            robot.odometry.updateLocalPosition();
            robot.odometry.updateGlobalPosition();
        }
        stopDrivetrain();
    }

    public NOdoLocation findError(NOdoLocation goal) {
        //Subtracts one location from another location
        //I was told not to make a method for this (Blame ben and jamie)
        NOdoLocation error = new NOdoLocation(
                goal.x - robot.odometry.position.x,
                goal.y - robot.odometry.position.y,
                rotationError(goal.rot, robot.odometry.position.rot)
        );

        double magnitude = Math.hypot(-error.x, error.y);
        double robotheading = Math.atan2(error.x, error.y);

        double forwardError = Math.cos(robotheading-Math.toRadians(robot.odometry.position.rot))*magnitude;
        double strafeError = Math.sin(robotheading-Math.toRadians(robot.odometry.position.rot))*magnitude;

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
