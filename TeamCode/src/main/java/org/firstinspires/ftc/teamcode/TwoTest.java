package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class TwoTest extends Mechanism {

    public TwoTest(DcMotorEx motor, DcMotorEx motor2) {
        super();
        motors.add(motor);
        motors.add(motor2);
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
    if (gp1.a) {
        motors.get(0).setPower(1);
    } else {
        motors.get(0).setPower(0);
    }

    if (gp1.b) {
        motors.get(1).setPower(1);
    } else {
        motors.get(1).setPower(0);
    }
    }

    @Override
    public void write() {

    }
}
