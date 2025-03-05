package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.arm.MoveArmToScoreCommand;
import frc.robot.commands.elevator.DeployElevatorL4Command;
import frc.robot.subsystems.Arm;

public class ElavateAndRotate extends ParallelCommandGroup{
    public ElavateAndRotate(Elevator elevator, Arm arm, 
    double elevatorTargetHeight, 
    double armTargetAngle,
    double elevatorThreshold) {
    addCommands(
        // Move the elevator to its target height immediately.
        new DeployElevatorL4Command(elevator),
            // Delay the arm movement until the elevator has reached the threshold.
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> elevator.getPosition() >= elevatorThreshold),
                new MoveArmToScoreCommand(arm)
            )
        );
    }

}
