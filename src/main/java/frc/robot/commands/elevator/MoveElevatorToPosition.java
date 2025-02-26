package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToPosition extends Command {
    private final Elevator elevator;
    private final double targetPosition;

    public MoveElevatorToPosition(Elevator elevator, double targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.goToPosition(targetPosition);
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