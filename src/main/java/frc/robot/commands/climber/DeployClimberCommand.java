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
    public void initialize() {
        // Move climber to deploy position
        climber.goToPosition(300.0);
    }

    @Override
    public boolean isFinished() {
        // Finish when climber reaches target position
        return climber.atPosition(300.0);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop climber when command ends
        climber.stop();
    }
}