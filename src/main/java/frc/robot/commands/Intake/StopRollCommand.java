package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoll;

public class StopRollCommand extends Command {
    private final IntakeRoll roll;
    public StopRollCommand(IntakeRoll roll){
        this.roll = roll;
        addRequirements(roll);
    }

    @Override
    public void execute() {
        roll.stop();
    }

    @Override
    public boolean isFinished() {
       return true;
    }
}
