package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveClimberToPosition extends Command {
    private final Climber climber;
    private final Double targetPosition;

    public MoveClimberToPosition(Climber climber, Double targetPosition) {
        this.climber = climber;
        this.targetPosition = targetPosition;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        double currentTarget = targetPosition;
        climber.goToPosition(currentTarget);
    }

    @Override
    public boolean isFinished() {
        // return false;
       return climber.atPosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
} 