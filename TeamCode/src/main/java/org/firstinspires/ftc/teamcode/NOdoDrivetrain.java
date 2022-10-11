package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AutoStuff.Variables;

/*
 * This class declares the drivetrain mechanism, sends data from the controller to the robot and
 * uses that data to set the motor powers
 */
public class NOdoDrivetrain extends Mechanism {
    //Declares variables
    public Robot robot; //declares a new instance of Robot
    public double[] motorPowers; //declares an array of motor powers
    public int[] motorIndices; //declares a new array of motor indices
//    public Telemetry telemetry; //declares a new instance of Telemetry
//    public Telemetry dashboardtelemetry;
    public PIDloops loops;
    private FtcDashboard dashboard;
    public NOdoLocation location;
    public Odometry odo;

    private double robotheading;
    private double magnitude;

//    private double DMPX = 0;
//    private double DMPY = 0;
//    private double DMPROT = 0;
    /*
    Declares instances of Location to move the robot forward, backward, left, right, clockwise,
    counterclockwise, and to the center of the field
     */

    //Spin PID variables
    public double spinError;
    public double previousSpinError = 20;

    private double lastForwardError; //Most recent forward error
    private double lastSidewaysError; //Most recent sideways error
    private double lastRotationError; //Most recent rotation error


    //Declares a new instance of location to store x and y errors
    public NOdoLocation error = new NOdoLocation();

    public double[] integralValues = new double[3];
    public double sinrang = 0;
    public double cosrang = 0;
    public double pow = 0;

    //Constructs a drivetrain object with parameters of the robot, motor numbers, telemetry, and 3 servos
    public NOdoDrivetrain(Robot bot, @NonNull int[] motorNumbers/*, Telemetry T, Servo SDrive1, Servo SDrive2, Servo SDrive3*/) {
        DcMotorEx motorPlaceholder;
        robot = bot;
        motorIndices = motorNumbers;
        //telemetry = T;

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        dashboardtelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        dashboard.updateConfig();
        odo = bot.odometry;
//        dashboard = FtcDashboard.getInstance();
//        dashboardtelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        /*
        servos.add(SDrive1);
        servos.add(SDrive2);
        servos.add(SDrive3);
        */


        for (int motNum : motorNumbers) {
            motorPlaceholder = robot.motors.get(motNum);
            motorPlaceholder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.add(motorPlaceholder);
        }
        motorPowers = new double[]{0, 0, 0, 0};
    }


    //Sets the motorNumbers array based on input from joysticks
    public void determineMotorPowers (Gamepad driverPad){
        double dpadVal=0;

        double P = Math.hypot(-driverPad.left_stick_x, -driverPad.left_stick_y);
        double robotAngle = Math.atan2(-driverPad.left_stick_y, -driverPad.left_stick_x);
        double rightX = driverPad.right_stick_x+dpadVal;

        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v1 = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontRight
        final double v2 = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontLeft
        final double v3 = (-P * sinRAngle) - (P * cosRAngle) - rightX; //backLeft
        final double v4 = (-P * sinRAngle) + (P * cosRAngle) + rightX; //backRight

        if (driverPad.right_bumper) {
            motorPowers[0] = v1*0.3;
            motorPowers[1] = v2*0.3;
            motorPowers[2] = v3*0.3;
            motorPowers[3] = v4*0.3;
        } else {
            motorPowers[0] = v1;
            motorPowers[1] = v2;
            motorPowers[2] = -v3;
            motorPowers[3] = -v4;
        }
    }

    public void determineMotorPowers(double x, double y, double rot) {
        //Power
        double P = Math.hypot(-x, y);
        //The angle that the robot is in right now
        double robotAngle = Math.atan2(y, -x);
        //The value that figures out the rotation that you want to go to
        double rightX = rot;

        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);
        cosrang = cosRAngle;
        sinrang = sinRAngle;
        pow = P;

        final double v1 = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontRight
        final double v2 = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontLeft
        final double v3 = (-P * sinRAngle) + (P * cosRAngle) + rightX;  //backRight
        final double v4 = (-P * sinRAngle) - (P * cosRAngle) - rightX;  //backLeft

        motorPowers[0] = v1; motorPowers[1] = v2; motorPowers[2] = -v3; motorPowers[3] = -v4;
    }

    //Updates data for Telemetry, motor powers, and servo movements
    public void update (Gamepad gp1, Gamepad gp2) {
/*
        if (gp1.b) {
            odoUp();
        } else if (gp1.back && !gp1.start) {
            odoDown();
        } else if (gp1.back && gp1.start) {
            robot.odometry.reset();
        }
        if (gp1.right_bumper && gp1.dpad_up) {
            altMode = true;
        }

        if (gp1.right_bumper && gp1.dpad_down) {
            altMode = false;
        }

        if(gp1.dpad_up){ //precision movement forward, very slow
            DMPY = DMPY + 0.45;
        }
        if(gp1.dpad_down){ //precision movement backward, very slow
            DMPY = DMPY - 0.45;
        }
        if(gp1.dpad_left) {
            DMPROT = DMPROT - 0.4;
        }
        if(gp1.dpad_right) {
            DMPROT = DMPROT + 0.4;
        }
        if(gp1.right_bumper){
            DMPX= DMPX + 0.55;
        }
        if(gp1.left_bumper){
            DMPX= DMPX - 0.55;
        }

        if (DMPX != 0 || DMPY != 0 || DMPROT != 0) {
            determineMotorPowers(DMPX,DMPY,DMPROT);
            DMPX = 0;
            DMPY = 0;
            DMPROT = 0;
        }
*/
        determineMotorPowers(gp1);
    }


    //Sets the motor powers based on the determineMotorPowers() method that was run in the update() method
    public void write () {
        int i = 0;
        for (DcMotorEx motor : motors.subList(motorIndices[0], motorIndices[2] + 1)) {
            motor.setPower(motorPowers[i]);
            i++;
        }
        robot.odometry.updatePosition();
        //Sets all telemetry for the drivetrain
//        telemetry.addLine("Motor Powers");
//        dashboardtelemetry.addData("Front Right Power", motorPowers[0]);
//        dashboardtelemetry.addData("Front Left Power", motorPowers[1]);
//        dashboardtelemetry.addData("Back Right Power", motorPowers[2]);
//        dashboardtelemetry.addData("Back Left Power", motorPowers[3]);
//        dashboardtelemetry.addData("ErrorX", + error.getLocation(0));
//        dashboardtelemetry.addData("ErrorY", + error.getLocation(2));
//        dashboardtelemetry.addData("ErrorRotation", + error.getLocation(2));
//        telemetry.update();
//        dashboardtelemetry.update();
    }


    //Moves to robot to the target position within a set amount of time
    public void moveToPosition(NOdoLocation goalPos, double xTolerance, double yTolerance, double rotTolerance,int maxTime) {
        integralValues = new double[3];
        error = findError(goalPos);
        double startTime = robot.getTimeMS();
        while ((robot.getTimeMS() - startTime < maxTime) && robot.linoop.opModeIsActive() && (Math.abs(error.getLocation(0)) > xTolerance || Math.abs(error.getLocation(1)) > yTolerance || Math.abs(error.getLocation(2)) > rotTolerance)) {
            error = findError(goalPos);
            write();
            robot.odometry.updatePosition();
//            dashboardTelemetry.addData("x-error",error.getLocation(0) );
//            dashboardTelemetry.addData("y-error",error.getLocation(1) );
//            dashboardTelemetry.addData("r-error",error.getLocation(2) );
//             dashboardTelemetry.update();
        }
        stopDrivetrain();
    }

    public void moveToPosition(NOdoLocation goalPos, double xTolerance, double yTolerance, double rotTolerance) {
        integralValues = new double[3];
        error = findError(goalPos);
        while (robot.linoop.opModeIsActive() && (Math.abs(error.getLocation(0)) > xTolerance || Math.abs(error.getLocation(1)) > yTolerance || Math.abs(error.getLocation(2)) > rotTolerance)) {
            error = findError(goalPos);
            write();
            robot.odometry.updatePosition();
//            dashboardTelemetry.addData("x-error",error.getLocation(0) );
//            dashboardTelemetry.addData("y-error",error.getLocation(1) );
//            dashboardTelemetry.addData("r-error",error.getLocation(2) );
            // dashboardTelemetry.update();
        }
        stopDrivetrain();
    }

    public void moveToPositionMod(NOdoLocation goalPos, double xTolerance, double yTolerance, double rotTolerance, int maxTime, double mod) {
        integralValues = new double[3];
        error = findErrorMod(goalPos, mod);
        double startTime = robot.getTimeMS();
        while ((robot.getTimeMS() - startTime < maxTime) &&robot.linoop.opModeIsActive()&&(Math.abs(error.getLocation(0)) > xTolerance || Math.abs(error.getLocation(1)) > yTolerance || Math.abs(error.getLocation(2)) > rotTolerance)) {
            error = findErrorMod(goalPos, mod);
            write();
            robot.odometry.updatePosition();
//            dashboardTelemetry.addData("x-error",error.getLocation(0) );
//            dashboardTelemetry.addData("y-error",error.getLocation(1) );
//            dashboardTelemetry.addData("r-error",error.getLocation(2) );
//             dashboardTelemetry.update();
        }
        stopDrivetrain();
    }

    public void moveToPositionMod(NOdoLocation goalPos, double xTolerance, double yTolerance, double rotTolerance, double mod) {
        integralValues = new double[3];
        error = findErrorMod(goalPos, mod);
        while (robot.linoop.opModeIsActive()&&(Math.abs(error.getLocation(0)) > xTolerance || Math.abs(error.getLocation(1)) > yTolerance || Math.abs(error.getLocation(2)) > rotTolerance)) {
            error = findErrorMod(goalPos, mod);
            write();
            robot.odometry.updatePosition();
//            dashboardTelemetry.addData("x-error",error.getLocation(0) );
//            dashboardTelemetry.addData("y-error",error.getLocation(1) );
//            dashboardTelemetry.addData("r-error",error.getLocation(2) );
            // dashboardTelemetry.update();
        }
        stopDrivetrain();
    }

    //Finds location error
    public  NOdoLocation findError( NOdoLocation goalPos) {
         NOdoLocation error = new NOdoLocation(
                goalPos.getLocation(0)-robot.odometry.realMaybe.getLocation(0),
                goalPos.getLocation(1) - robot.odometry.realMaybe.getLocation(1),
                rotationError(goalPos.getLocation(2), robot.odometry.realMaybe.getLocation(2)));
        //this is to change the global xy error into robot specific error
        magnitude = Math.hypot(-error.getLocation(0),error.getLocation(1));
        robotheading = robot.odometry.getPosition().getLocation(2)- Math.atan2(error.getLocation(1),-error.getLocation(0));
        robotheading = Math.atan2(error.getLocation(0),error.getLocation(1));

        double forwardError = (Math.cos(robotheading-Math.toRadians(robot.odometry.realMaybe.getLocation(2)))*magnitude);
        double strafeError = (Math.sin(robotheading-Math.toRadians(robot.odometry.realMaybe.getLocation(2)))*magnitude);

        if(Math.abs(Variables.kfP*forwardError + Variables.kfI*integralValues[0] + Variables.kfD * (forwardError- lastForwardError))<1)
            integralValues[0]= integralValues[0]+forwardError;
        if(Math.abs(Variables.ksP*strafeError + Variables.ksI*integralValues[1] + Variables.ksD * (strafeError - lastSidewaysError))<1)
            integralValues[1]= integralValues[1]+strafeError;
        if(Math.abs(Variables.krP*error.getLocation(2) + Variables.krI*integralValues[2] + Variables.krD * (error.getLocation(2) - lastRotationError))<1)
            integralValues[2]= integralValues[2]+error.getLocation(2);

        double forwardPow = (Variables.kfP*forwardError+ Variables.kfI*integralValues[0] + Variables.kfD * (forwardError - lastForwardError));
        double sidePow = (Variables.ksP*strafeError + Variables.ksI*integralValues[1] + Variables.ksD * (strafeError - lastSidewaysError)) ;
        double rotPow = -(Variables.krP *error.getLocation(2) + Variables.krI*integralValues[2] +Variables.krD * ( error.getLocation(2) - lastRotationError));

        lastForwardError = forwardPow;
        lastSidewaysError = sidePow;
        lastRotationError = rotPow;

        determineMotorPowers(sidePow,forwardPow,rotPow);
        return error;
    }

    public  NOdoLocation findErrorMod( NOdoLocation goalPos, double mod) {
         NOdoLocation error = new NOdoLocation(
                goalPos.getLocation(0)-robot.odometry.realMaybe.getLocation(0),
                goalPos.getLocation(1) - robot.odometry.realMaybe.getLocation(1),
                rotationError(goalPos.getLocation(2), robot.odometry.realMaybe.getLocation(2)));
        //this is to change the global xy error into robot specific error
        magnitude = Math.hypot(-error.getLocation(0),error.getLocation(1));
        robotheading = robot.odometry.getPosition().getLocation(2)- Math.atan2(error.getLocation(1),-error.getLocation(0));
        robotheading = Math.atan2(error.getLocation(0),error.getLocation(1));

        double forwardError = Math.cos(robotheading-Math.toRadians(robot.odometry.realMaybe.getLocation(2)))*magnitude;
        double strafeError = Math.sin(robotheading-Math.toRadians(robot.odometry.realMaybe.getLocation(2)))*magnitude;

        if(Math.abs(Variables.kfP*forwardError + Variables.kfI*integralValues[0] + Variables.kfD * (forwardError- lastForwardError))<1)
            integralValues[0]= integralValues[0]+forwardError;
        if(Math.abs(Variables.ksP*strafeError + Variables.ksI*integralValues[1] + Variables.ksD * (strafeError - lastSidewaysError))<1)
            integralValues[1]= integralValues[1]+strafeError;
        if(Math.abs(Variables.krP*error.getLocation(2) + Variables.krI*integralValues[2] + Variables.krD * (error.getLocation(2) - lastRotationError))<1)
            integralValues[2]= integralValues[2]+error.getLocation(2);

        double forwardPow = mod*((Variables.kfP*forwardError+ Variables.kfI*integralValues[0] + Variables.kfD * (forwardError - lastForwardError)));
        double sidePow = mod*((Variables.ksP*strafeError + Variables.ksI*integralValues[1] + Variables.ksD * (strafeError - lastSidewaysError)));
        double rotPow = -(Variables.krP *error.getLocation(2) + Variables.krI*integralValues[2] +Variables.krD * ( error.getLocation(2) - lastRotationError));

        lastForwardError = forwardPow;
        lastSidewaysError = sidePow;
        lastRotationError = rotPow;

        determineMotorPowers(sidePow,forwardPow,rotPow);
        return error;
    }

    /*
     * Determines powers for each motor
     */
    public void fieldRelDetermineMotorPowers(double x, double y, double rot) {
        //P is the power
        //robotAngle is the angle to which you want to go
        //rightX is the value of the x-axis from the right joystick

        double P = Math.hypot(-x, y);
        double robotAngle = Math.atan2(y, -x);
        double rightX = rot;

        double sinRAngle = Math.sin(robotAngle-Math.toRadians(robot.odometry.getPosition().getLocation(2)));
        double cosRAngle = 1.2*Math.cos(robotAngle-Math.toRadians(robot.odometry.getPosition().getLocation(2)));
//        telemetry.addData("robot angle",robotAngle);
//        telemetry.addData("sin angle",sinRAngle);
//        telemetry.addData("cos angle",cosRAngle);

        final double frPower = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontRight
        final double flPower = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontLeft
        final double brPower = (P * sinRAngle) + (P * cosRAngle) + rightX;  //backRight
        final double blPower = (P * sinRAngle) - (P * cosRAngle) - rightX;  //backLeft


        motorPowers[0] = frPower; motorPowers[1] = flPower; motorPowers[2] = brPower; motorPowers[3] = blPower;

    }

    /*
     * Calculates the error for the rotation
     * @param goal  where we need to rotate to
     * @param current where we are
     * @return how far off the rotation was
     */
    public double rotationError(double goal, double current){
        spinError = goal - current ;

        if(spinError > 180) {
            spinError = spinError - 360;
        } else if (spinError < -180) {
            spinError = spinError + 360;
        }
        else if(spinError==180){
            spinError = previousSpinError;
        }
        return (spinError);
    }

    /*
     * Stops the drivetrain
     */
    public void stopDrivetrain(){
        determineMotorPowers(0,0,0);
        this.write();
    }

//    public void odoUp () {
//        servos.get(0).setPosition(0.45);
//        servos.get(1).setPosition(0.3);
//        servos.get(2).setPosition(0.45);
//    }

//    public void odoDown () {
//        servos.get(0).setPosition(0.71);//R
//        servos.get(1).setPosition(0.64);//M
//        servos.get(2).setPosition(.17);//L
//    }
}