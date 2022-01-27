package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

/*
 * Picks up and scores the cap using an arm
 */
public class Cap extends Mechanism {
    //Fields
    //FtcDashboard dashboard;
    //Telemetry dashboardTelemetry;
    /*Length of the cap arm*/
    private final float armLength = 1;
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
        servos.get(0).setPosition(1);
    }
    /*
     * Moves the arm to the scoring/scoring height
     */
    public void moveToStoringHeight(){
        servos.get(0).setPosition(0);
    }
    public void pickUpElement(){
        servos.get(0).setPosition(0);
        try {
            wait(3000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        servo.setPosition(1);
    }

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
    }

    /*
     * Updates every cycle
     */
    @Override
    public void write() {

    }
}