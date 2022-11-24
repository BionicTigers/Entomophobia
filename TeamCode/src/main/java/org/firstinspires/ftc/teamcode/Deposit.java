//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.Servo;
//
///**
// * The deposit uses a servo to deposit cones picked up from the transfer.
// * @author Jack Gerber
// */
//public class Deposit extends Mechanism {
//    public Servo servo;
//    public boolean open = false;
//    public boolean close = true;
//    public boolean pressed = false;
//    /**
//     * Adds values to the variables
//     * @param s imported servo
//     */
//    public Deposit (Servo s) {
//        super();
//        servo = s;
//        servos.add(servo);
//    }
//    /**
//     * Takes inputs from controllers and translates them into robot movement
//     * @param gp1 the gamepad controlling the drivetrain
//     * @param gp2 the gamepad controlling the lift
//     */
//    public void update(Gamepad gp1, Gamepad gp2) {
//        if(gp2.a && open && !pressed) {
//            servos.get(0).setPosition(0.52);
//            open = false;
//            close = true;
//        }
//        else if(gp2.a && close && !pressed){
//            servos.get(0).setPosition(0.4);
//            open = true;
//            close = false;
//        }
//        pressed = gp2.a;
//    }
//
//
////FOR SHOWCASE NIGHT USE ONLY
////    public void update(Gamepad gp1, Gamepad gp2) {
////        if(gp1.a && open && !pressed) {
////            servos.get(0).setPosition(0.52);
////            open = false;
////            close = true;
////        }
////        else if(gp1.a && close && !pressed){
////            servos.get(0).setPosition(0.4);
////            open = true;
////            close = false;
////        }
////        pressed = gp2.a;
////    }
//    public void write() {
//
//    }
//}
