
package org.firstinspires.ftc.teamcode.RookieTraining;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Mechanism;

public class Trigger {
    private Boolean held;
    private Boolean pos;

    public void update(float a) {
        if (a > 0.3) {
            if (!held) {
                held = true;
                pos = !pos;
            }
        } else {
            held = false;
        }

        write();
    }

    public void write() {
        if (pos) {
            //do stuff here
            System.out.print("Position 1");
        } else {
            //do stuff here
            System.out.print("Position 0");
        }
    }

    public static void main(String[] args) {
        Trigger t = new Trigger();
        t.update(1);
        t.update(0);
        t.update(1);
        t.update(0);
    }
}