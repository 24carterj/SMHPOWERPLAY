package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TwoLiftSubsystem;

public class TwoLiftUp extends CommandBase {

    private final TwoLiftSubsystem lifts;

    public TwoLiftUp(TwoLiftSubsystem l) {
        lifts = l;
        addRequirements(lifts);

    }

    @Override
    public void execute() {
        lifts.up();
    }
    /*
    @Override
    public boolean isFinished() {
        return lift.stopConditions();
    }
     */
}
