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

    /*
     * Cap constructor; makes a new instance of cap
     */
    public Cap(Servo capServo){
        super();
        getServos().add(capServo);
    }

    /*
     * Moves the arm to the intake height
     */
    public void moveToIntakeHeight(){
        servos.get(0).setPosition(.98);
    }
    /*
     * Moves the arm to the scoring/scoring height
     */
    public void moveToStoringHeight(){
        servos.get(0).setPosition(0.05);
    }
    public void moveToScoringHeight(){
        servos.get(0).setPosition(0.61);
    }
    public void moveToDoubleScoringHeight(){
        servos.get(0).setPosition(0.75);
    }
    public void moveToBalanceHeight() {servos.get(0).setPosition(.68);}

    /*
     * Returns arm servo position
     */
    public double getArmHeight(){
        return servo.getPosition();
    }

    /*
     * Updates every cycle
     */
    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if(gp2.dpad_up)
            moveToStoringHeight();
        if(gp2.dpad_down)
            moveToIntakeHeight();
        if(gp2.x){
            moveToDoubleScoringHeight();
        }
        if(gp2.y){
            moveToScoringHeight();
        }
        if(gp2.right_stick_button){
            moveToBalanceHeight();
        }
    }

    /*
     * Updates every cycle
     */
    @Override
    public void write() {

    }
}