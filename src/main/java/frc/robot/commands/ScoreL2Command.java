package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ScoreL2Command extends ParallelCommandGroup{
    public ScoreL2Command(Elevator elevator, Arm arm){
        addCommands(
                new MovePCM(arm, -227),
                new SequentialCommandGroup(
                    new WaitUntilCommand(arm.aboveIntake),
                    new MovePCM(elevator, 0)
                ) 
            )
        ;
    }
}
