package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoll;

public class RunRollCommand extends Command {
    private final IntakeRoll roll;
    public RunRollCommand(IntakeRoll roll){
        this.roll = roll;
        addRequirements(roll);
    }

    @Override
    public void execute() {
        roll.setSpeed(-0.5);
    }
}
