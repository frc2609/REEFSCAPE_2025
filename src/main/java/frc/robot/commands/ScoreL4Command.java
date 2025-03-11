package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ScoreL4Command extends ParallelCommandGroup{
    public ScoreL4Command(Elevator elevator, Arm arm){
        addCommands(
            new MovePCM(elevator, 37),
            new SequentialCommandGroup(
                new WaitCommand(.5),
                new MovePCM(arm, -222)
            ),
            Commands.print("level4")
            
        );
    }
}