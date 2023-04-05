package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TwoLiftSubsystem;

public class TwoLiftDown extends CommandBase {

    private final TwoLiftSubsystem lifts;

    public TwoLiftDown(TwoLiftSubsystem l) {
        lifts = l;
        addRequirements(lifts);
    }

    @Override
    public void execute() {
        lifts.down();
    }
    /*
    @Override
    public boolean isFinished() {
        return lift.stopConditions();
    }
     */
}
