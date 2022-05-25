package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    public Telemetry telemetry;
    public boolean altMode = false;
    public double verticalPosition = 0.3;

    /*
     * Cap constructor; makes a new instance of cap
     */
    public Cap(CRServo capCR, Servo capN, Telemetry T){
        super();
        getCRServos().add(capCR);
        getServos().add(capN);
        telemetry = T;
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
            if (gp1.left_stick_y >= 0.3) {
                getCRServos().get(0).setPower(-1);
            } else if (gp1.left_stick_y <= -0.3) {
                getCRServos().get(0).setPower(1);
            } else {
                getCRServos().get(0).setPower(0);
            }
            verticalPosition = verticalPosition - (gp1.right_stick_y * .005);
            if(verticalPosition > 1) verticalPosition = 1;
            if(verticalPosition < -1) verticalPosition = -1;
            servos.get(0).setPosition(verticalPosition);
        }
        else{
            servos.get(0).setPosition(.5);
        }
        telemetry.addData("VerticalCapPos", verticalPosition);

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