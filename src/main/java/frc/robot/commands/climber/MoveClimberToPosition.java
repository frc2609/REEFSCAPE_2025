package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveClimberToPosition extends Command {
    private final Climber climber;
    private final Supplier<Double> targetPositionSupplier;

    public MoveClimberToPosition(Climber climber, Supplier<Double> targetPositionSupplier) {
        this.climber = climber;
        this.targetPositionSupplier = targetPositionSupplier;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        double currentTarget = targetPositionSupplier.get();
        climber.goToPosition(currentTarget);
    }

    @Override
    public boolean isFinished() {
        return false;
      //  return climber.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
} 