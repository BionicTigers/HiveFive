package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

/*
 * Picks up and scores the cap using an arm
 * @author Jack 2
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

    /**
     * Cap constructor
     * Makes a new instance of cap
     * @param capServo    servo used to control the cap arm
     */
    public Cap(Servo capServo){
        super();
        servo = capServo;
        getServos().add(capServo);
    }


    /*
     * Moves the arm to the intake height
     */
    public void moveToIntakeHeight(){
        servo.setPosition(0);
    }
    /*
     * Moves the arm to the scoring/scoring height
     */
    public void moveToStoringHeight(){
        servo.setPosition(1);
    }
    public void pickUpElement(){
        servo.setPosition(0);
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