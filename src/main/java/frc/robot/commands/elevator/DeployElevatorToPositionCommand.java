package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class DeployElevatorToPositionCommand extends Command {
    private final Elevator elevator;
    private final double position;

    public DeployElevatorToPositionCommand(Elevator elevator, double position) {
        this.elevator = elevator;
        this.position = position;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.goToPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop elevator when command ends
        elevator.stop();
    }
}