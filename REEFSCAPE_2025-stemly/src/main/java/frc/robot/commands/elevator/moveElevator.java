package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class moveElevator extends Command {
    private Elevator elevator;
    public moveElevator(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.goToPosition(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
