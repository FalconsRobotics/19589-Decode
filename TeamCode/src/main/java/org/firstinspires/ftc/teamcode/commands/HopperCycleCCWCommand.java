package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;

/**
 * Command used to rotate the hopper counterclockwise to cycle the balls.
 */
public class HopperCycleCCWCommand extends CommandBase {
    private final HopperSubsystem hopper;

    /**
     * Initializes the HopperCycleCCWCommand.
     * @param suppliedHopper Uses the hopper from your running OpMode.
     */
    public HopperCycleCCWCommand(HopperSubsystem suppliedHopper) {
        this.hopper = suppliedHopper;

        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        this.hopper.moveOnePosition(1);
    }
}
