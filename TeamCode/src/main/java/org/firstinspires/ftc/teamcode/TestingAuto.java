//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@Autonomous (name="TestingAuto", group="autonomous")
//public class TestingAuto extends LinearOpMode {
//    public Robot robot;
//    public Drivetrain drivetrain;
//    public String[] motorNames = {"frontRight","frontLeft","backLeft","backRight"};
//    public int[] motorNumbers = {0, 1, 2, 3}; //creates motor numbers array
//    public Claw claw;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = new Robot(this);
//        drivetrain = new Drivetrain(robot, motorNumbers, telemetry);
//        claw = new Claw(hardwareMap.get(Servo.class, "clawl"), hardwareMap.get(Servo.class, "clawr"));
//
//        claw.initopen();
//
//        waitForStart();
//
//        //FORWARD, NEEDED FOR ALL THREE POSITIONS
//        drivetrain.motors.get(0).setPower(0.5);
//        drivetrain.motors.get(1).setPower(0.5);
//        drivetrain.motors.get(2).setPower(0.5);
//        drivetrain.motors.get(3).setPower(0.5);
//        sleep(1000);
//        drivetrain.motors.get(0).setPower(0);
//        drivetrain.motors.get(1).setPower(0);
//        drivetrain.motors.get(2).setPower(0);
//        drivetrain.motors.get(3).setPower(0);
//        sleep(1000);
//
//        //LEFT
////        drivetrain.motors.get(0).setPower(0.5);
////        drivetrain.motors.get(1).setPower(-0.5);
////        drivetrain.motors.get(2).setPower(0.5);
////        drivetrain.motors.get(3).setPower(-0.5);
////        sleep(1100);
////        drivetrain.motors.get(0).setPower(0);
////        drivetrain.motors.get(1).setPower(0);
////        drivetrain.motors.get(2).setPower(0);
////        drivetrain.motors.get(3).setPower(0);
////
////        //RIGHT
////        drivetrain.motors.get(0).setPower(-0.5);
////        drivetrain.motors.get(1).setPower(0.5);
////        drivetrain.motors.get(2).setPower(-0.5);
////        drivetrain.motors.get(3).setPower(0.5);
////        sleep(1100);
////        drivetrain.motors.get(0).setPower(0);
////        drivetrain.motors.get(1).setPower(0);
////        drivetrain.motors.get(2).setPower(0);
////        drivetrain.motors.get(3).setPower(0);
//    }
//}
