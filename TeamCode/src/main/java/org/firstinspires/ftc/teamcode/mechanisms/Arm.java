package org.firstinspires.ftc.teamcode.mechanisms;

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

public class Arm extends Mechanism {
    public Telemetry telemetry;

    public enum State {
        IDLE,
        MOVING
    }
    public State currentState;

    private int targetTicks = 0;

    private int junkTicks;
    public PID pid;
    public static final double kP = 1, kI = 0, kD = 0;


    public ControlHub controlHub;
    public int encoderPos = 0;

    public CRServo left;
    public CRServo right;

    public Arm (CRServo l, CRServo r, HardwareMap hardwareMap, Telemetry T) {
        super();
        //Declares the left motor
        crServos.add(l);
        left = l;
        //Declares the right motor
        crServos.add(r);
        right = r;

        //Flips the right servo so it rotates the right way
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        //Declares telemetry
        telemetry = T;

        //Declares Control Hub
        controlHub = new ControlHub(hardwareMap, hardwareMap.get(LynxDcMotorController.class, "Control Hub"));

        controlHub.refreshBulkData();
        junkTicks = controlHub.getEncoderTicks(3);

        pid = new PID(kP, kI, kP, 0, 6144, -1, 1);
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        //Resets lift encoder position
        encoderPos = controlHub.getEncoderTicks(3);

        //Defaults to a 0 power move
        move(0);
        //Moves the arm upward slowly
        if (gp2.right_bumper) {
            move(0.4);
        }
        //Moves the arm downward slowly
        if (gp2.left_bumper) {
            move(-0.4);
        }
        //Moves the arm upward quickly
        if (gp2.dpad_up || gp2.dpad_left || gp2.dpad_down) {
            move(1);
        }
        //Moves the arm downward quickly
        if (gp2.dpad_right) {
            move(-1);
        }
    }

    @Override
    public void write() {
        //📝
        controlHub.refreshBulkData();
        int currentTicks = controlHub.getEncoderTicks(3) - junkTicks;
        double output = pid.calculate(targetTicks, currentTicks);

        //Moves the servos
        left.setPower(output);
        right.setPower(output);
        if (currentTicks - targetTicks <= 20) {
            currentState = State.IDLE;
        }
        telemetry.addData("Arm ticks: ", controlHub.getEncoderTicks(3));
        telemetry.addData("Arm degrees: ", controlHub.getEncoderTicks(3)/(8192/360));
        telemetry.addData("PID: ", output);
        telemetry.update();
    }

    //Moves the lift forward
    public void move(double degrees) {
        //0 degrees = storing position; 150 = scoring position
        //8192 ticks per revolution; 8192 / 360 = 22.756
        targetTicks = (int) Math.round(degrees * (8192 / 360));
        currentState = State.MOVING;
    }
}