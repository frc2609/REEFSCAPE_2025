package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ScoreL3Command extends ParallelCommandGroup{
    public ScoreL3Command(Elevator elevator, Arm arm){
        addCommands(
            new MovePCM(elevator, 12),
            new SequentialCommandGroup(
                new WaitCommand(.5),
                new MovePCM(arm, -222)
            )
        );
    }
}
