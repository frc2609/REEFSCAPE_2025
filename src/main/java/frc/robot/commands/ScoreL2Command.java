package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ScoreL2Command extends SequentialCommandGroup{
    public ScoreL2Command(Arm arm){
        addCommands(
            new MovePCM(arm, -227)
        );
    }

}
