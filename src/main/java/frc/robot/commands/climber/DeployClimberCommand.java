package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class DeployClimberCommand extends Command {
    private final Climber climber;

    public DeployClimberCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.goToPosition(180);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}