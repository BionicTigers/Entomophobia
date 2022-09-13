//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.internal.system.Deadline;
//
//
//import java.util.concurrent.TimeUnit;
//
///*
//This class calls the different mechanisms on the robot so that they can all be used based on the
//controller input
//*/
//
//@TeleOp (name = "TeleOpWorlds")
//public class TeleOpWorlds extends LinearOpMode{
//    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"}; //establishes motor names
//    public Drivetrain drivetrain; //declares drivetrain
//    public Robot robot; //declares robot
//    public Intake intake;
//    public Cap cap;
//    public Spinner spinner;
//    public Turret turret;
//    //public ColorSensor color;
//    //public RevBlinkinLedDriver blinkinLedDriver;
//    //RevBlinkinLedDriver.BlinkinPattern pattern;
//    //RevBlinkinLedDriver.BlinkinPattern pattern2;
//
//    public int currentPattern = 1;
//    private RevBlinkinLedDriver.BlinkinPattern endgamePattern;
//
//    public boolean hasFreight = false;
//
//    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array
//    private Deadline endgameFlashies = new Deadline(85, TimeUnit.SECONDS);
//    private Deadline endgameFlashiesOver = new Deadline(90,TimeUnit.SECONDS);
//
//    DistanceSensor distance;
//
//    public void runOpMode() {
//        robot = new Robot(this);
//        drivetrain = new Drivetrain(robot, motorNumbers, telemetry, hardwareMap.get(Servo.class, "SDriveL"), hardwareMap.get(Servo.class, "SDriveM"), hardwareMap.get(Servo.class, "SDriveR"));
//        distance = hardwareMap.get(DistanceSensor.class, "distance");
//        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"), hardwareMap.get(Servo.class, "intake2"), distance);
//        spinner = new Spinner(hardwareMap.get(DcMotorEx.class,"spinner"));
//        cap = new Cap(hardwareMap.get(CRServo.class, "cap1"), hardwareMap.get(Servo.class, "cap2"), telemetry);
//        turret = new Turret(hardwareMap.get(DcMotorEx.class, "turretSpin"), hardwareMap.get(DcMotorEx.class, "turretLift"), hardwareMap.get(Servo.class, "turretLeft"), hardwareMap.get(Servo.class, "turretRight"), telemetry);
//        //color = hardwareMap.get(ColorSensor.class, "color");
//        robot.initMotors(motorNames);
////        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
////        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;
////        pattern2 = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
////        endgamePattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE;
////        blinkinLedDriver.setPattern(pattern);
//
//
////        Deadline endgameFlashiesOver = new Deadline(90,TimeUnit.SECONDS);
////        Deadline endgameFlashies = new Deadline(80,TimeUnit.SECONDS);
//        //These lines set motors and servos to their default position once teleOp starts
//        waitForStart();
//        //output.servos.get(0).setPosition(.7);
//        drivetrain.odoUp();
//        Mechanism[] mechanisms = {intake, spinner, drivetrain, turret, robot.odometry, cap};
//
//
////        endgameFlashies.reset();
////        endgameFlashiesOver.reset();
//        //what runs constantly once play button is pressed
//        while(opModeIsActive()) {
//
//            if(distance.getDistance(DistanceUnit.CM) < 0.8) {
//                hasFreight = true;
//            } else {
//                hasFreight = false;
//            }
//
//            //drivetrain.dashboardtelemetry.addData("red", color.red()/55.0);
//            //drivetrain.dashboardtelemetry.addData("green", color.green()/55.0);
//            //drivetrain.dashboardtelemetry.addData("blue", color.blue()/55.0);
//            drivetrain.dashboardtelemetry.addData("Spinner Position", spinner.motors.get(0).getCurrentPosition());
//            drivetrain.dashboardtelemetry.addData("Spinner Speed", spinner.motors.get(0).getPower());
//            drivetrain.dashboardtelemetry.addData("Spinning?", spinner.spinning);
//            drivetrain.dashboardtelemetry.addData("deployed?", spinner.deployed);
//            drivetrain.dashboardtelemetry.addData("Distance (cm)", distance.getDistance(DistanceUnit.CM));
//            drivetrain.dashboardtelemetry.addData("Has freight? ", hasFreight);
//            drivetrain.dashboardtelemetry.addData("CapVertPos", cap.verticalPosition);
////            drivetrain.dashboardtelemetry.addData("endgameFlashies", endgameFlashies.hasExpired());
////            drivetrain.dashboardtelemetry.addData("endgameFlashiesOver? :(", endgameFlashiesOver.hasExpired());
////            if(endgameFlashies.hasExpired() && !endgameFlashiesOver.hasExpired() && currentPattern != 3){
////                currentPattern = 3;
////                blinkinLedDriver.setPattern(endgamePattern);
////            }
////            else if(color.green()/55.0 > 10.5 && currentPattern !=2 && (!endgameFlashies.hasExpired() || endgameFlashiesOver.hasExpired()))//green when nothing = 8.4
////            {
////                blinkinLedDriver.setPattern(pattern2);
////                currentPattern = 2;
////            }
////            else if(currentPattern != 1 && !(color.red()/55.0 > 5.5) && (!endgameFlashies.hasExpired() || endgameFlashiesOver.hasExpired()))
////            {
////                blinkinLedDriver.setPattern(pattern);
////                currentPattern = 1;
////            }
////            drive.update();
//
//            for (Mechanism mech : mechanisms) { //For each mechanism in the mechanism array
//                mech.update(gamepad1, gamepad2); //Run their respective update methods
//            }
//
//            for (Mechanism mech : mechanisms) { //For each mechanism in the mechanism array
//                mech.write(); //Run their respective write methods
//            }
//
//            drivetrain.dashboardtelemetry.addData("Vertical trim: ", turret.verticalTrim);
//            drivetrain.dashboardtelemetry.addData("Horizontal trim: ", turret.horizontalTrim);
//            drivetrain.dashboardtelemetry.addData("Spin trim: ", turret.spinTrim);
//            drivetrain.dashboardtelemetry.addData("Turret spin motor: ", turret.motors.get(0).getCurrentPosition());
//            drivetrain.dashboardtelemetry.addData("Turret lift motor: ", turret.motors.get(1).getCurrentPosition());
//            drivetrain.dashboardtelemetry.addData("Left servo: ", turret.servos.get(0).getPosition());
//            drivetrain.dashboardtelemetry.addData("Right servo: ", turret.servos.get(1).getPosition());
//            drivetrain.dashboardtelemetry.addData("Stick: ", gamepad1.right_stick_y);
//            drivetrain.dashboardtelemetry.addData("forward?", turret.forward);
//            drivetrain.dashboardtelemetry.addData("left?", turret.left);
//            drivetrain.dashboardtelemetry.addData("right?", turret.right);
//
//
//        }
//    }
//}
