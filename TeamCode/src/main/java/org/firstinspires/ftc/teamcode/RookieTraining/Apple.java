package org.firstinspires.ftc.teamcode.RookieTraining;

public class Apple {
    public String color;
    public int numBites;
    public int[] dimensions;
    public boolean isRotten;

    public Apple(String col, int nBruises, int[] d) {
        color = col;
        numBites = nBruises;
        dimensions = d;
        isRotten = false;
    }

    public Apple(String col, int nBruises, int[] d, boolean rot) {
        color = col;
        numBites = nBruises;
        dimensions = d;
        isRotten = rot;
    }

    public void bite() {
        numBites++;
    }
    public void bite(int b) {
        numBites += b;
    }

    public void rot() {
        color = "brown";
        isRotten = true;
    }

    public String getColor() {
        return color;
    }

    public void paint(String paintColor) {
        color = paintColor;
    }
}