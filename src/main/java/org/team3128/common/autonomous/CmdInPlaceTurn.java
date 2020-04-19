// Authors: Mika, Thomas, Mason, Sohan
package org.team3128.common.autonomous;

import org.team3128.common.drive.Drive;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.compbot.subsystems.FalconDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdInPlaceTurn extends CommandGroup {
    double startAngle, angle, timeoutMs, power;
    FalconDrive drive;
    Gyro gyro;

    public CmdInPlaceTurn(FalconDrive drive, Gyro gyro, double angle, double power, double timeoutMs) {
        this.drive = drive;
        this.angle = angle;
        this.gyro = gyro;
        this.power = power;
        this.timeoutMs = timeoutMs;
    }

    @Override
    protected void initialize() {
        startAngle = drive.getAngle();
        if (angle > 0) {
            drive.setWheelPower(new DriveSignal(-power, power));
        }
        else if (angle < 0) {
            drive.setWheelPower(new DriveSignal(power, -power));
        }
    }

    @Override
    protected boolean isFinished() {
        return drive.getAngle() - startAngle >= angle;
    }

    @Override
    protected void end() {
        drive.setWheelPower(new DriveSignal(0, 0));
    }
}