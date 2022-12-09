package org.firstinspires.ftc.teamcode;

public class Cluster {
    private int x;
    private int y;
    private int c;

    private int low;
    private int high;
    private int left;
    private int right;

    public Cluster(int _x, int _y) {
        x = _x;
        y = _y;
        left = x;
        right = x;
        low = y;
        high = y;
        c = 1;
    }

    public void clean() {
        x /= c;
        y /= c;
    }
    public void add(int _x, int _y) {

        if (_y > high) {high = _y;}
        if (_y < low ) {low  = _y;}
        if (_x < left) {left = _x;}
        if (_x > right) {right = _x;}

        x += _x;
        y += _y;
        c++;
    }

    public int getX() {
        return x;
    }
    public int getY() {
        return y;
    }
    public int getCount() {
        return c;
    }
    public int getLow() {
        return low;
    }
    public int getHigh() {
        return high;
    }
    public int getLeft() {
        return left;
    }
    public int getRight() {
        return right;
    }
}