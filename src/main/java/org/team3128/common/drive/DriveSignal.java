package org.team3128.common.drive;

public class DriveSignal {
    /*
     * Inches per second for speed
     */
    public double leftVelocity;
    public double rightVelocity;
    public double leftAcc;
    public double rightAcc;

    public DriveSignal(double left, double right) {
        this(left, 0, right, 0);
    }

    public DriveSignal(double left, double leftAcc, double right, double rightAcc) {
        leftVelocity = left;
        this.leftAcc = leftAcc;
        rightVelocity = right;
        this.rightAcc = rightAcc;
    }
}