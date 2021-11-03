package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

/**
 * Picks up and scores the cap using an arm
 * @author Jack 2
 */
public class Cap extends Mechanism {
    //Fields
    //FtcDashboard dashboard;
    //Telemetry dashboardTelemetry;
    /**Length of the cap arm*/
    private final float armLength = 1;
    /**New instance of robot*/
    Robot robot;

    /**
     * Cap constructor
     * Makes a new instance of cap
     * @param armServo    servo used to control the arm
     */
    public Cap(Robot robot, Servo armServo){
        super();
        getServos().add(armServo);
    }

    /**
     * Moves the arm to the intake height
     */
    public void moveToIntakeHeight(){
        servos.get(0).setPosition(0.125);
    }

    /**
     * Moves the arm to the scoring/scoring height
     */
    public void moveToStoringHeight(){
        servos.get(0).setPosition(0.375);
    }

    /**
     * Returns arm servo position
     */
    public double getArmHeight(){
        return servos.get(0).getPosition();
    }

    /**
     * Updates every cycle
     */
    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if(gp2.dpad_up)
            moveToStoringHeight();
        if(gp2.dpad_down || gp2.dpad_up && gp2.left_bumper)
            moveToIntakeHeight();
    }

    /**
     * Updates every cycle
     */
    @Override
    public void write() {

    }
}