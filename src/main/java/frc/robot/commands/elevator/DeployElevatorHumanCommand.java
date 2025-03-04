package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class DeployElevatorHumanCommand extends Command {
    private final Elevator elevator;

    public DeployElevatorHumanCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.goToPosition(340);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop elevator when command ends
        elevator.stop();
    }
}