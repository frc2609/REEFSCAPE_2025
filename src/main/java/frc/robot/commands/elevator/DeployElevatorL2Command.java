package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class DeployElevatorL2Command extends Command {
    private final Elevator elevator;

    public DeployElevatorL2Command(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.goToPosition(10);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop elevator when command ends
        elevator.stop();
    }
}