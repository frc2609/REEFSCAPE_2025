package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveClimberToPosition extends Command {
    private final Climber climber;
    private final double targetPosition;

    public MoveClimberToPosition(Climber climber, double targetPosition) {
        this.climber = climber;
        this.targetPosition = targetPosition;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.goToPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return climber.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
} 