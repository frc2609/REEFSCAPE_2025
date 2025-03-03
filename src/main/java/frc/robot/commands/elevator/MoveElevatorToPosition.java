package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToPosition extends Command {
    private final Elevator elevator;
    private final Supplier<Double> targetPositionSupplier;

    public MoveElevatorToPosition(Elevator elevator, Supplier<Double> targetPositionSupplier) {
        this.elevator = elevator;
        this.targetPositionSupplier = targetPositionSupplier;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double currentTarget = targetPositionSupplier.get();
        elevator.goToPosition(currentTarget);
    }

    // @Override
    // public boolean isFinished() {
    //     return elevator.atPosition();
    // }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
} 