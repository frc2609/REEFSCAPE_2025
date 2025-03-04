package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class DeployElevatorL3Command extends Command {
    private final Elevator elevator;

    public DeployElevatorL3Command(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.goToPosition(20);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop elevator when command ends
        elevator.stop();
    }
}