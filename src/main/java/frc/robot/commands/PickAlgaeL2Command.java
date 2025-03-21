package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.gripper.GripCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class PickAlgaeL2Command extends SequentialCommandGroup{
    public PickAlgaeL2Command(Elevator elevator, Arm arm, Gripper gripper, Trigger confirmTrigger) {
        addCommands(
            new WaitUntilCommand(confirmTrigger),
            new ParallelCommandGroup(
                new MovePCM(elevator, 9),
                new GripCommand(gripper),
                new MovePCM(arm, 40)
            ),
            Commands.idle(elevator, arm, gripper)

        );
    }
}
