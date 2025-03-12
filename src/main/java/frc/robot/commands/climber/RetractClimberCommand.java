package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class RetractClimberCommand extends Command {
    private final Climber climber;

    public RetractClimberCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.goToPosition(15.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}