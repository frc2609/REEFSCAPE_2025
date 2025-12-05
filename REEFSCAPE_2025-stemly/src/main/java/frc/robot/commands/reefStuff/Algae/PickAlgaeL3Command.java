package frc.robot.commands.reefStuff.Algae;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.gripper.GripCommand;
import frc.robot.commands.gripper.ReleaseGripperCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class PickAlgaeL3Command extends SequentialCommandGroup{
    public PickAlgaeL3Command(Elevator elevator, Arm arm, Gripper gripper, Trigger confirmTrigger) {
        addCommands(

            new MovePCM(elevator, 20),
            new WaitUntilCommand(confirmTrigger),
            new ParallelCommandGroup(
                new GripCommand(gripper),
                new MovePCM(arm, 60)
            ),
            new WaitUntilCommand(confirmTrigger),
            new ReleaseGripperCommand(gripper),

            Commands.idle(elevator, arm, gripper)
        );
    }
}
