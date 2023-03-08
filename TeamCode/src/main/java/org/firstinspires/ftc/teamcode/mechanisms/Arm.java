package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.teleop.Robot;
import org.firstinspires.ftc.teamcode.util.ControlHub;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.teamcode.util.PID;

import java.util.concurrent.TimeUnit;

@Config
public class Arm extends Mechanism {
    public Telemetry telemetry;

    public enum State {
        IDLE,
        MOVING
    }
    public State currentState;

    private double targetDegrees = 0;

    public PID pid;
    public static double kP = 1.25, kI = 0, kD = 0;
    public static double biasMultiplier = .12;

    private final int angleOffset = 8;

    private ControlHub controlHub;

    public CRServo left;
    public CRServo right;

    public double setPosition = 26;

    public double lastTime;

    public Arm (CRServo left, CRServo right, HardwareMap hardwareMap, Telemetry telemetry) {
        this.left = left;
        this.right = right;

        this.telemetry = telemetry;

        //Makes the left servo rotate the opposite direction
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        //Allows access to encoder ticks and removes old ticks
        controlHub = new ControlHub(hardwareMap, hardwareMap.get(LynxDcMotorController.class, "Control Hub"));
        controlHub.setJunkTicks(3);

        move(-30);

        pid = new PID(kP, kI, kD, 0, 270, -1, 1);

        lastTime = System.currentTimeMillis();
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {

        //PID CODE

//        //Change in time (in seconds) so we can have accurate position changes
        double dt = (System.currentTimeMillis() - lastTime) / 1000;
        lastTime = System.currentTimeMillis();

        //Moves the arm upward slowly
        if (gp2.right_bumper) {
            setPosition += 150 * dt;
        }
        //Moves the arm downward slowly
        if (gp2.left_bumper) {
            setPosition -= 150 * dt;
        }
        //Moves the arm upward quickly
        if (gp2.dpad_up || gp2.dpad_left) {
            setPosition = 260;
        }
        if(gp2.dpad_down) {
            setPosition = 117.5;
        }
        //Moves the arm downward quickly
        if (gp2.dpad_right) {
            setPosition = 0;
        }

        setPosition = Math.max(-30, Math.min(270, setPosition));

        move(setPosition);

        //NON PID CODE

//        if (gp2.dpad_up || gp2.dpad_left || gp2.dpad_down || gp2.right_bumper) {
//            left.setPower(1);
//            right.setPower(1);
//        } else if (gp2.dpad_right || gp2.left_bumper) {
//            left.setPower(-1);
//            right.setPower(-1);
//        } else {
//            left.setPower(0);
//            right.setPower(0);
//        }
    }

    @Override
    public void write() {

        //PID CODE

        controlHub.refreshBulkData();

        //Translate current ticks into degrees and localize to 0
        double currentDegrees = ((double) controlHub.getEncoderTicks(3) / 8192 * 360) + angleOffset;

        //Calculate the bias to add to the output then calculate the output of the PID
        double bias = Math.sin(Math.toRadians(currentDegrees)) * biasMultiplier;
        double output = pid.calculate(targetDegrees, currentDegrees) + bias;

        //Check if the arm is within a range to see if the move is completed
        if (Math.abs(currentDegrees - targetDegrees) < 4) {
            currentState = State.IDLE;
        }

        //Check if the arm is in a deadzone where no power should be applied (lower and upper bounds)
        if ((currentDegrees < 10 && targetDegrees < 10) || (currentDegrees > 260 && targetDegrees > 260)) {
            output = 0;
        }

        //Moves the servos
        left.setPower(output);
        right.setPower(output);


        //NON PID CODE
        //Literally none lmao
    }

    //Controls the Position to where the arm moves
    //Degrees is from 0 - 270 (starting - scoring)
    public void move(double degrees) {
        //Clamp degrees to 0 and 270 then localize to 0
        targetDegrees = Math.max(0, Math.min(270, degrees)) - angleOffset;
        currentState = State.MOVING;
    }
}