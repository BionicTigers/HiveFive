package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

/*
 * Picks up and scores the cap using an arm
 */
public class Cap extends Mechanism {
    //Fields
    //FtcDashboard dashboard;
    //Telemetry dashboardTelemetry;
    /*New instance of robot*/
    public Robot robot;
    public Servo servo;

    public boolean altMode = false;
    public double verticalPosition = 0;

    /*
     * Cap constructor; makes a new instance of cap
     */
    public Cap(CRServo capCR, Servo capN){
        super();
        getCRServos().add(capCR);
        getServos().add(capN);
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp1.right_bumper && gp1.dpad_up) {
            altMode = true;
        }
        if (gp1.right_bumper && gp1.dpad_down) {
            altMode = false;
        }

        if (altMode) {
            getCRServos().get(0).setPower(gp1.left_stick_y);
            verticalPosition = verticalPosition + (gp1.right_stick_y*0.25);
            getServos().get(0).setPosition(verticalPosition);
        }
    }

    @Override
    public void write() {

    }
//
//    /*
//     * Moves the arm to the intake height
//     */
//    public void moveToIntakeHeight(){
//        servos.get(0).setPosition(0.95);
//    }
//    /*
//     * Moves the arm to the scoring/scoring height
//     */
//    public void moveToStoringHeight(){
//        servos.get(0).setPosition(0.1);
//    }
//    public void moveToScoringHeight() { servos.get(0).setPosition(0.55); } // prescore
//    public void moveToScoringHeight2(){
//        servos.get(0).setPosition(0.71);
//    } // knock the cap off
//    public void pickUpElement(){
//        servos.get(0).setPosition(0);
//        try {
//            wait(3000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        servo.setPosition(1);
//    }
//
//    /*
//     * Returns arm servo position
//     */
//    public double getArmHeight(){
//        return servo.getPosition();
//    }
//
//    /*
//     * Updates every cycle
//     */
//    @Override
//    public void update(Gamepad gp1, Gamepad gp2) {
//        if(gp2.dpad_up)
//            moveToStoringHeight();
//        if(gp2.dpad_down)
//            moveToIntakeHeight();
//        if(gp2.x){
//            moveToScoringHeight2();
//        }
//        if(gp2.y){
//            moveToScoringHeight();
//        }
//        if (gp1.right_bumper && gp1.dpad_up) {
//            altMode = true;
//        }
//
//        if (gp1.right_bumper && gp1.dpad_down) {
//            altMode = false;
//        }
//    }
//
//    /*
//     * Updates every cycle
//     */
//    @Override
//    public void write() {
//
//    }
}