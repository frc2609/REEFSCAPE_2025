package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.gripper.GripCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class CoralHandOff extends SequentialCommandGroup{
    public CoralHandOff(Elevator elevator, Arm arm, Gripper gripper ) {
        addCommands(
            new MovePCM(arm, 15).withTimeout(0.5),

            new ParallelCommandGroup(
                new MovePCM(elevator, 1).withTimeout(0.5),
                new MovePCM(arm, -20).withTimeout(0.5),
                new GripCommand(gripper).withTimeout(0.5)
            )
        );
    }
}
