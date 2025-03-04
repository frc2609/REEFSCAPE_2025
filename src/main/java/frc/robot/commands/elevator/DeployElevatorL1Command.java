package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class DeployElevatorL1Command extends Command {
    private final Elevator elevator;

    public DeployElevatorL1Command(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.goToPosition(5);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop elevator when command ends
        elevator.stop();
    }
}