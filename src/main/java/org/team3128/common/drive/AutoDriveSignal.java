package org.team3128.common.drive;

public class AutoDriveSignal {
    public DriveSignal command;
    public boolean isDone;

    public AutoDriveSignal(DriveSignal command, boolean isDone) {
        this.command = command;
        this.isDone = isDone;
    }
}