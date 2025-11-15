package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;

/**
 * Command used to cycle the hopper clockwise to extake the balls.
 */
public class HopperCycleCWCommand extends CommandBase {
    private final HopperSubsystem hopper;

    /**
     * Initializes the HopperCycleCWCommand.
     * @param suppliedHopper Uses the hopper from your running OpMode.
     */
    public HopperCycleCWCommand(HopperSubsystem suppliedHopper) {
        this.hopper = suppliedHopper;

        addRequirements(hopper);
    }

    @Override
    public void execute() {
        this.hopper.extakeOneTick();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
