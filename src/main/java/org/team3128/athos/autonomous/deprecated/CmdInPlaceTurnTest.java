package org.team3128.athos.autonomous.deprecated;

import org.team3128.common.utility.enums.Direction;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdInPlaceTurnTest extends CommandGroup {
    public CmdInPlaceTurnTest() {
        addSequential(SRXTankDrive.getInstance().new CmdInPlaceTurn(360, Direction.RIGHT, 1.0, 10000));
    }
}