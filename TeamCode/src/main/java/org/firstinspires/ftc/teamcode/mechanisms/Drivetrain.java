package org.firstinspires.ftc.teamcode.mechanisms;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.OdoPods;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.teamcode.teleop.Robot;
import org.firstinspires.ftc.teamcode.util.Location;
import org.firstinspires.ftc.teamcode.util.IndependentPID;
import org.firstinspires.ftc.teamcode.util.OdoPods;
import org.firstinspires.ftc.teamcode.util.Variables;

/*
 * This class declares the drivetrain mechanism, sends data from the controller to the robot and
 * uses that data to set the motor powers
 */
public class Drivetrain extends Mechanism {
    //Declares variables
    public Robot robot; //declares a new instance of Robot
    public double[] motorPowers; //declares an array of motor powers
    public Telemetry telemetry; //declares a new instance of Telemetry
    public MultipleTelemetry dashboardTelemetry;
    private FtcDashboard dashboard;
    public Location location;
    public Odometry odo;

    public IndependentPID xIndependentPid;
    public IndependentPID yIndependentPid;
    public IndependentPID rIndependentPid;

    private double robotheading;
    private double magnitude;

    private double DMPX = 0;
    private double DMPY = 0;
    private double DMPROT = 0;

    //Spin PID variables
    public double spinError;
    public double previousSpinError = 20;

    private double lastForwardError; //Most recent forward error
    private double lastSidewaysError; //Most recent sideways error
    private double lastRotationError; //Most recent rotation error


    //Declares a new instance of location to store x and y errors
    public Location error;

    public double[] integralValues = new double[3];
    public double sinrang = 0;
    public double cosrang = 0;
    public double pow = 0;

    public enum State {
        IDLE,
        MOVE_TO_POSITION
    };

    //Used for Move to Position
    private double startTime;
    private double maxTime;
    private Location tolerance;
    private Location goalPosition;
    private double modifier = 0;
    public State currentState;

    public Drivetrain(Robot bot, int[] _, Telemetry T, Servo LeftOdo, Servo RightOdo, Servo BackOdo) {
        DcMotorEx motorPlaceholder;
        robot = bot;
        telemetry = T;

        currentState = State.IDLE;

        xIndependentPid = new IndependentPID(1, 0.1, 0, -100, 10000);
        yIndependentPid = new IndependentPID(1, 0.1, 0, -100, 10000);
        rIndependentPid = new IndependentPID(1, 0.1, 0, -10000, 10000);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        odo = bot.odometry;
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        servos.add(LeftOdo);
        servos.add(RightOdo);
        servos.add(BackOdo);

        for (int motNum = 0; motNum<4; motNum++) {
            motorPlaceholder = robot.motors.get(motNum);
            motorPlaceholder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.add(motorPlaceholder);
        }
        motorPowers = new double[]{0, 0, 0, 0};
    }

    //Constructs a drivetrain object with parameters of the robot, motor numbers, telemetry, and 3 servos
    public Drivetrain(Robot bot, Telemetry T, Servo LeftOdo, Servo RightOdo, Servo BackOdo) {
        DcMotorEx motorPlaceholder;
        robot = bot;
        telemetry = T;

        currentState = State.IDLE;

        xIndependentPid = new IndependentPID(1, 0.1, 0, -100, 10000);
        yIndependentPid = new IndependentPID(1, 0.1, 0, -100, 10000);
        rIndependentPid = new IndependentPID(1, 0.1, 0, -10000, 10000);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        odo = bot.odometry;
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        servos.add(LeftOdo);
        servos.add(RightOdo);
        servos.add(BackOdo);

        for (int motNum = 0; motNum<4; motNum++) {
            motorPlaceholder = robot.motors.get(motNum);
            motorPlaceholder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.add(motorPlaceholder);
        }
        motorPowers = new double[]{0, 0, 0, 0};
    }

    //Sets the motorNumbers array based on input from joysticks
    public void determineMotorPowers (Gamepad gp1){
        double dpadVal=0;

        double P = Math.hypot(-gp1.left_stick_x, -gp1.left_stick_y);
        double robotAngle = Math.atan2(-gp1.left_stick_y, -gp1.left_stick_x);
        double rightX = gp1.right_stick_x+dpadVal;

        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v1 = (P * sinRAngle) + (P * cosRAngle) - rightX;  //frontRight
        final double v2 = (P * sinRAngle) - (P * cosRAngle) + rightX;  //frontLeft
        final double v3 = (-P * sinRAngle) - (P * cosRAngle) - rightX; //backLeft
        final double v4 = (-P * sinRAngle) + (P * cosRAngle) + rightX; //backRight

        if (gp1.right_bumper) {
            motorPowers[0] = v1*0.3;
            motorPowers[1] = v2*0.3;
            motorPowers[2] = -v3*0.3;
            motorPowers[3] = -v4*0.3;
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
        final double v3 = (-P * sinRAngle) - (P * cosRAngle) - rightX; //backLeft
        final double v4 = (-P * sinRAngle) + (P * cosRAngle) + rightX; //backRight



        motorPowers[0] = v1; motorPowers[1] = v2; motorPowers[2] = -v3; motorPowers[3] = -v4;
    }

    public void determineMotorPowers(double x, double y, double rot, double mod) {
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
        final double v3 = (-P * sinRAngle) - (P * cosRAngle) - rightX; //backLeft
        final double v4 = (-P * sinRAngle) + (P * cosRAngle) + rightX; //backRight

        motorPowers[0] = v1 * mod; motorPowers[1] = v2 * mod; motorPowers[2] = -v3 * mod; motorPowers[3] = -v4 * mod;
    }

    //Updates data for Telemetry, motor powers, and servo movements
    public void update (Gamepad gp1, Gamepad gp2) {
        if (gp1.left_stick_button) {
            odoUp();
        } else if (gp1.right_stick_button) {
//            odoDown();
        }

//        if (gp1.right_bumper && gp1.dpad_up) {
//            altMode = true;
//        }
//
//        if (gp1.right_bumper && gp1.dpad_down) {
//            altMode = false;
//        }

        if(gp1.dpad_up){ //precision movement forward, very slow
            DMPY = DMPY + 0.55;
        }
        if(gp1.dpad_down){ //precision movement backward, very slow
            DMPY = DMPY - 0.55;
        }
        if(gp1.dpad_left) {
            DMPX = DMPX - 0.45;
        }
        if(gp1.dpad_right) {
            DMPX = DMPX + 0.45;
        }
        if(gp1.right_bumper){
            DMPROT= DMPROT + 0.4;
        }
        if(gp1.left_bumper){
            DMPROT= DMPROT - 0.4;
        }

        if (DMPX != 0 || DMPY != 0 || DMPROT != 0) {
            currentState = State.IDLE;
            determineMotorPowers(DMPX,DMPY,DMPROT);
            DMPX = 0;
            DMPY = 0;
            DMPROT = 0;
        } else if (gp1.left_stick_y > 0.3 || gp1.left_stick_y < -0.3
                || gp1.left_stick_x > 0.3 || gp1.left_stick_x < -0.3
                || gp1.right_stick_x > 0.3 || gp1.right_stick_x < -0.3){
            currentState = State.IDLE;
            determineMotorPowers(gp1);
        } else if (currentState == State.IDLE) {
            determineMotorPowers(0,0,0);
        }
    }



    //Sets the motor powers based on the determineMotorPowers() method that was run in the update() method
    @Override
    public void write() {
        if (currentState == State.MOVE_TO_POSITION) {
            error = findErrorMod(goalPosition, this.modifier);
            if ((robot.getTimeMS() - startTime < maxTime)
                    && (Math.abs(error.getLocation(0)) > tolerance.getLocation(0)
                    || Math.abs(error.getLocation(1)) > tolerance.getLocation(1)
                    || Math.abs(error.getLocation(2)) > tolerance.getLocation(2)))
            {
                error = findErrorMod(goalPosition, this.modifier);

                robot.odometry.updateLocalPosition();
                robot.odometry.updateGlobalPosition();

                dashboardTelemetry.addData("x-error",error.getLocation(0));
                dashboardTelemetry.addData("y-error",error.getLocation(1));
                dashboardTelemetry.addData("r-error",error.getLocation(2));
                dashboardTelemetry.update();
            } else {
                currentState = State.IDLE;
                determineMotorPowers(0,0,0);
            }
        }

        double highest = 0;
        for (double power : motorPowers) {
            highest = Math.max(power, highest);
        }

        int i = 0;
        for (DcMotorEx motor : motors) {
            //Balance the motor powers
            if (highest > 1) {
                motorPowers[i] *= 1 / highest;
                telemetry.addData("Ratio", 1 / highest);
            }
            motor.setPower(motorPowers[i]);
            i++;
        }
    }


    //Moves to robot to the target position within a set amount of time
    public void moveToPosition(Location goalPos, double xTolerance, double yTolerance, double rotTolerance, int maxTime) {
        this.goalPosition = goalPos;
        this.tolerance = new Location(xTolerance, yTolerance, rotTolerance);
        this.maxTime = maxTime;
        this.startTime = robot.getTimeMS();
        this.modifier = 1;

        integralValues = new double[3];

        this.currentState = Drivetrain.State.MOVE_TO_POSITION;
    }

//    public void moveToPosition(NOdoLocation goalPos, double xTolerance, double yTolerance, double rotTolerance) {
//        integralValues = new double[3];
//        error = findError(goalPos);
//        while (robot.linoop.opModeIsActive() && (Math.abs(error.getLocation(0)) > xTolerance || Math.abs(error.getLocation(1)) > yTolerance || Math.abs(error.getLocation(2)) > rotTolerance)) {
//            error = findError(goalPos);
//            write();
//            robot.odometry.updatePosition();
////            dashboardTelemetry.addData("x-error",error.getLocation(0) );
////            dashboardTelemetry.addData("y-error",error.getLocation(1) );
////            dashboardTelemetry.addData("r-error",error.getLocation(2) );
//            // dashboardTelemetry.update();
//        }
//        stopDrivetrain();
//    }
//
    public void moveToPositionMod(Location goalPos, double xTolerance, double yTolerance, double rotTolerance, double mod, int maxTime) {
        this.goalPosition = goalPos;
        this.tolerance = new Location(xTolerance, yTolerance, rotTolerance);
        this.maxTime = maxTime;
        this.startTime = robot.getTimeMS();
        this.modifier = mod;

        integralValues = new double[3];

        this.currentState = Drivetrain.State.MOVE_TO_POSITION;
    }
//
//    public void moveToPositionMod(NOdoLocation goalPos, double xTolerance, double yTolerance, double rotTolerance, double mod) {
//        integralValues = new double[3];
//        error = findErrorMod(goalPos, mod);
//        while (robot.linoop.opModeIsActive()&&(Math.abs(error.getLocation(0)) > xTolerance || Math.abs(error.getLocation(1)) > yTolerance || Math.abs(error.getLocation(2)) > rotTolerance)) {
//            error = findErrorMod(goalPos, mod);
//            write();
//            robot.odometry.updatePosition();
////            dashboardTelemetry.addData("x-error",error.getLocation(0) );
////            dashboardTelemetry.addData("y-error",error.getLocation(1) );
////            dashboardTelemetry.addData("r-error",error.getLocation(2) );
//            // dashboardTelemetry.update();
//        }
//        stopDrivetrain();
//    }
//
//    //Finds location error
    public Location findError(Location goalPos) {
         Location error = new Location(
                goalPos.getLocation(0)-robot.odometry.position.getLocation(0),
                goalPos.getLocation(1) - robot.odometry.position.getLocation(1),
                rotationError(goalPos.getLocation(2), robot.odometry.position.getLocation(2)));
        //this is to change the global xy error into robot specific error
        magnitude = Math.hypot(-error.getLocation(0),error.getLocation(1));
        robotheading = robot.odometry.getPosition().getLocation(2)- Math.atan2(error.getLocation(1),-error.getLocation(0));
        robotheading = Math.atan2(error.getLocation(0),error.getLocation(1));

        double forwardError = (Math.sin(robotheading-Math.toRadians(robot.odometry.position.getLocation(2)))*magnitude);
        double strafeError = (Math.cos(robotheading-Math.toRadians(robot.odometry.position.getLocation(2)))*magnitude);

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

    public Location findErrorMod(Location goalPos, double mod) {
         Location error = new Location(
                goalPos.getLocation(0) - robot.odometry.position.getLocation(0),
                goalPos.getLocation(1) - robot.odometry.position.getLocation(1),
                Math.toRadians(goalPos.getLocation(2)) - robot.odometry.position.getLocation(2));
        //this is to change the global xy error into robot specific error
        magnitude = Math.hypot(-error.getLocation(0),error.getLocation(1));
        robotheading = robot.odometry.position.getLocation(2)- Math.atan2(error.getLocation(1),-error.getLocation(0));
        robotheading = Math.atan2(error.getLocation(0),error.getLocation(1));

        double forwardError = Math.cos(robotheading-robot.odometry.position.getLocation(2))*magnitude;
        double strafeError = Math.sin(robotheading-robot.odometry.position.getLocation(2))*magnitude;

        if(Math.abs(Variables.kfP*forwardError + Variables.kfI*integralValues[0] + Variables.kfD * (forwardError- lastForwardError))<1)
            integralValues[0] += forwardError;
        if(Math.abs(Variables.ksP*strafeError + Variables.ksI*integralValues[1] + Variables.ksD * (strafeError - lastSidewaysError))<1)
            integralValues[1] += strafeError;
        if(Math.abs(Variables.krP * error.getLocation(2) + Variables.krI*integralValues[2] + Variables.krD * (error.getLocation(2) - lastRotationError))<1)
            integralValues[2] += error.getLocation(2);

        double forwardPow = ((Variables.kfP*forwardError+ Variables.kfI*integralValues[0] + Variables.kfD * (forwardError - lastForwardError)));
        double sidePow = ((Variables.ksP*strafeError + Variables.ksI*integralValues[1] + Variables.ksD * (strafeError - lastSidewaysError)));
        double rotPow = (Variables.krP * (error.getLocation(2)) + Variables.krI*integralValues[2] +Variables.krD * ((error.getLocation(2)) - lastRotationError));

        lastForwardError = forwardPow;
        lastSidewaysError = sidePow;
        lastRotationError = rotPow;

        determineMotorPowers(sidePow,forwardPow,rotPow, mod);
        telemetry.addData("data!", forwardError+" "+sidePow+" "+rotPow);
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

    //REAL BOT ODO POSITIONS

    public void odoUp () {
        servos.get(0).setPosition(OdoPods.LEFT_UP);//L
        servos.get(1).setPosition(OdoPods.MIDDLE_UP);//M
        servos.get(2).setPosition(OdoPods.RIGHT_UP);//R
    }

    public void odoDown () {
        servos.get(0).setPosition(OdoPods.LEFT_DOWN);//L
        servos.get(1).setPosition(OdoPods.MIDDLE_DOWN);//M
        servos.get(2).setPosition(OdoPods.RIGHT_DOWN);//R
    }

    //TEST BOT ODO POSITIONS

//    public void odoUp () {
//        servos.get(0).setPosition(1);//L
//        servos.get(1).setPosition(0.5);//M
//        servos.get(2).setPosition(1);//R
//    }
//
//    public void odoDown () {
//        servos.get(0).setPosition(0.3);//L
//        servos.get(1).setPosition(0.1);//M
//        servos.get(2).setPosition(0.05);//R
//    }
}