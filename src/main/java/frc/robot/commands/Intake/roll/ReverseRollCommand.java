package frc.robot.commands.Intake.roll;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoll;

public class ReverseRollCommand extends Command {
    private final IntakeRoll roll;
    public ReverseRollCommand(IntakeRoll roll){
        this.roll = roll;
        addRequirements(roll);
    }

    @Override
    public void execute() {
        roll.setSpeed(0.5);
    }
}
