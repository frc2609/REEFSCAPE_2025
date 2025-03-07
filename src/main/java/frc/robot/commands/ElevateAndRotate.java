package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.DeployElevatorToPositionCommand;
import frc.robot.subsystems.Arm;

public class ElevateAndRotate extends ParallelCommandGroup{
    public ElevateAndRotate(Elevator elevator, Arm arm, 
    double elevatorTargetHeight, 
    double armTargetAngle,
    double elevatorThreshold) {
    addCommands(
        // Move the elevator to its target height immediately.
        new DeployElevatorToPositionCommand(elevator, 10),
            // Delay the arm movement until the elevator has reached the threshold.
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> elevator.getPosition() >= elevatorThreshold),
                new MoveArmToPosition(arm, 20)
            )
        );
    }

}
