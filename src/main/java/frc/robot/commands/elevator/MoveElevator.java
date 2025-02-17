package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends SequentialCommandGroup {
    private static final double TOP_POSITION = 1.0;     // Adjust based on your elevator's range
    private static final double MIDDLE_POSITION = 2.0;  // Adjust based on your elevator's range
    private static final double HOME_POSITION = 0.0;    // Bottom position

    public MoveElevator(Elevator elevator) {
        addCommands(
            new MoveElevatorToPosition(elevator, TOP_POSITION)//.withTimeout(2),
            // new WaitCommand(0.2),
            // new MoveElevatorToPosition(elevator, MIDDLE_POSITION).withTimeout(2),
            // new WaitCommand(0.2),
            // new MoveElevatorToPosition(elevator, HOME_POSITION).withTimeout(2)
        );
    }
} 