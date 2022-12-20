package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;

public class NewDrivetrain extends Mechanism {
    private Telemetry telemetry;
    private HashMap<String, Servo> odoServos;
    private ArrayList<DcMotorEx> driveMotors;

    private double[] motorPowers;

    public NewDrivetrain(NOdoRobot robot, Telemetry T, HashMap<String, Servo> OM) {
        telemetry = T;
        odoServos = OM;
        driveMotors = robot.motors;

        motorPowers = new double[4];

        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }

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

    public void odoUp () {
        servos.get(0).setPosition(0.45);
        servos.get(1).setPosition(0.3);
        servos.get(2).setPosition(0.45);
    }

    public void odoDown () {
        servos.get(0).setPosition(1);//R
        servos.get(1).setPosition(0.64);//M
        servos.get(2).setPosition(.17);//L
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
