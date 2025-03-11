package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;

public class ScoreL2Command extends SequentialCommandGroup{
    public ScoreL2Command(Arm arm){
        addCommands(
            new MovePCM(arm, -227)
        );
    }

}
