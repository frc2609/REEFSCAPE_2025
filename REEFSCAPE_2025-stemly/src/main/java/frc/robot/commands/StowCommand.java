package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class StowCommand extends ParallelCommandGroup {
    public StowCommand(Elevator elevator, Arm arm){
        addCommands(
            new MovePCM(elevator, 8.5),
            new SequentialCommandGroup(
                new WaitUntilCommand(elevator.aboveIntake),
                new ParallelCommandGroup(
                    new MovePCM(arm, 0),
                    new MovePCM(elevator, 0) 
                )
            )
        );
    }
}
