package org.firstinspires.ftc.teamcode;

public class Delay implements Runnable {
    private Thread t;
    private float time;
    private Callback callback;

    Delay(long ms, Callback cb) {
        time = ms;
        callback = cb;
    }

    interface Callback {
        void call();
    }

    @Override
    public void run() {
        try {
            Thread.sleep((long) time);
            callback.call();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void start() {
        if (t == null) {
            t = new Thread(this);
            t.start();
        }
    }

    public void await() {
        if (t != null) {
            try {
                t.join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}