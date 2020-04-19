package org.team3128.common.autonomous.primitives;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * This command is constructed with a group of commands. They will be run in
 * parallel when the command is invoked.
 */
public class CmdRunInParallel extends CommandGroup {

    public CmdRunInParallel(Command... commands) {
        if (commands == null || commands.length < 1) {
            throw new IllegalArgumentException("You must provide at least one command!");
        }
        // addSequential(commands[0]);
        for (int index = 0; index < commands.length; ++index) {
            addParallel(commands[index]);
        }
    }
}
