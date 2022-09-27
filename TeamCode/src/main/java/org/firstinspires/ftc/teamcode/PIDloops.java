package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;

public class PIDloops extends Drivetrain {

    public PID dtPIDx;
    public PID dtPIDz;
    public PID dtPIDr;
    public PID[] dtPIDs = new PID[3];
    private Location forward = new Location (0, 0, 100, 0);
    private Location backward = new Location (0, 0, -500, 0);
    private Location left = new Location (-500, 0, 0, 0);
    private Location right = new Location (500, 0, 0, 0);
    private Location clockwise = new Location (0, 0, 0, 90);
    private Location counterclockwise = new Location (0, 0, 0, 270);
    private Location center = new Location (0, 0, 0, 0);
    private String cap;

    public PIDloops(Robot bot, int[] motorNumbers, Telemetry telem/*, Servo servo1, Servo servo2, Servo servo3*/, PID pidx, PID pidz, PID pidr){
        super(bot, motorNumbers, telem/*, servo1, servo2, servo3*/);
        dtPIDx = pidx;
        dtPIDz = pidz;
        dtPIDr = pidr;
        dtPIDs[0] = dtPIDx;
        dtPIDs[1] = dtPIDz;
        dtPIDs[2] = dtPIDr;
    }

    public Location determineError(Location location) {
        error.setLocation(0, location.getLocation(0) - robot.odometry.realMaybe.getLocation(0));
        error.setLocation(2, location.getLocation(2) - robot.odometry.realMaybe.getLocation(2));
        //error.setLocation(3, rotationError(location.getLocation(3), robot.odometry.realMaybe.getLocation(3)));

        return error;
    }

    public void pidGo(Location location) {
        int i = 0;
        determineError(location);
        for(PID pid: dtPIDs) {
            pid.findOutput(error.getLocation(i));
            i++;
            if(i == 1) {
                i++;
            }
        }
        //fieldRelDetermineMotorPowers(dtPIDx.getCv(), dtPIDz.getCv(), dtPIDr.getCv()); // just switched x and z
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if(gp1.dpad_up){ //precision movement forward, very slow
            pidGo(forward);
            //write();
            cap = "dUp";
        } else if(gp1.dpad_down){ //precision movement backward, very slow
            pidGo(backward);
            cap = "dDown";
        } else if(gp1.dpad_left) {
            pidGo(left);
            cap = "dLeft";
        } else if(gp1.dpad_right) {
            pidGo(right);
            cap = "dRight";
        } else if (gp1.left_bumper) {
            pidGo(counterclockwise);
            cap = "lBump";
        } else if (gp1.right_bumper) {
            pidGo(clockwise);
            write();
            cap = "rBump";
        } else if (gp1.right_trigger > .5) {
            pidGo(new Location(0, 0, 0,(float) Variables.setPointRotation));
            write();
            cap = "rTrigger";
        } else {
            for (PID pid : dtPIDs) {
                pid.pidStart();
            }
            determineMotorPowers(robot.gamepad1);
            cap = "nada";
        }

//        dashboardTelemetry.addData("r error", error.getLocation(3));
//        dashboardTelemetry.addData("r cv", dtPIDr.getCv());
//        dashboardTelemetry.update();
//
//        telemetree.addData("button", cap);
//        telemetree.addLine("POWERS");
//        telemetree.addData("x", dtPIDx.getCv());
//        telemetree.addData("z", dtPIDz.getCv());
//        telemetree.addData("r", dtPIDr.getCv());
//
//        telemetree.addData("fr", String.valueOf(motorPowers[0]));
//        telemetree.addData("fl", String.valueOf(motorPowers[1]));
//        telemetree.addData("br", String.valueOf(motorPowers[2]));
//        telemetree.addData("bl", String.valueOf(motorPowers[3]));
//
//        telemetree.addLine("ERRORS");
//        telemetree.addData("xe", String.valueOf(error.getLocation(0)));
//        telemetree.addData("ze", String.valueOf(error.getLocation(2)));
//        telemetree.addData("re", String.valueOf(error.getLocation(3)));
//        telemetree.addLine("CURRENT POSITION");
//        telemetree.addData("x", String.valueOf(robot.odometry.getPosition().getLocation(0)));
//        telemetree.addData("z", String.valueOf(robot.odometry.getPosition().getLocation(2)));
//        telemetree.addData("r", String.valueOf(robot.odometry.getPosition().getLocation(3)));
//        telemetree.addLine("OTHER THINGS");
//        telemetree.addData("xdeltaE", dtPIDx.deltaE);
//        telemetree.addData("zdeltaE", dtPIDz.deltaE);
//        telemetree.addData("rdeltaE", dtPIDr.deltaE);
//        telemetree.addData("pidError", dtPIDx.getError());
//        telemetree.addData("lastError", dtPIDx.getLastError());
//        telemetree.addData("laLaError", dtPIDx.getLastLastError());
//        telemetree.addData("startTime", dtPIDx.startTime);
//        telemetree.addData("currentTime", dtPIDx.systemTime.now(SECONDS));
    }
}
