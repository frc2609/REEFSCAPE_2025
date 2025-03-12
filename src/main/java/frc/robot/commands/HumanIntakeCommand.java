package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.gripper.GripCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class HumanIntakeCommand extends ParallelCommandGroup {
    public HumanIntakeCommand(Arm arm, Gripper gripper, Elevator elevator){
        addCommands(
            new MovePCM (elevator, 10.5),
            new GripCommand(gripper),
            new SequentialCommandGroup(
                new WaitUntilCommand(elevator.aboveIntake),
                new MovePCM(arm, -72)
            )
        );
    }
}
