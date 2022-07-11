package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class PortTest extends Mechanism {
    private boolean activate = false;
    private int index = 0;
    private Telemetry telemetry;
    private ArrayList<Object> objects = new ArrayList<Object>();

    public PortTest(ArrayList<DcMotorEx> m, ArrayList<Object> s)
    {
        objects.addAll(m);
        objects.addAll(s);
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp1.left_bumper) {
            index = Math.max(0, index-1);
            telemetry.addData("Index", index);
        } else if (gp1.right_bumper) {
            index = Math.max(9, index+1);
            telemetry.addData("Index", index);
        }

        if (gp1.x) {
            activate = true;
        } else {
            activate = false;
        }
        telemetry.update();
    }

    @Override
    public void write() {
        Object thingToActivate = objects.get(index);

        if (activate) {
            if (thingToActivate instanceof DcMotorSimple) {
                ((DcMotorSimple) thingToActivate).setPower(1);
            } else if (thingToActivate instanceof Servo) {
                ((Servo) thingToActivate).setPosition(1);
            }
        } else {
            if (thingToActivate instanceof DcMotorSimple) {
                ((DcMotorSimple) thingToActivate).setPower(0);
            } else if (thingToActivate instanceof Servo) {
                ((Servo) thingToActivate).setPosition(0);
            }
        }
    }
}
