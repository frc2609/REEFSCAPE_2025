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
            new MovePCM(elevator, 36),
            new SequentialCommandGroup(
                new WaitCommand(.5),
                new MovePCM(arm, -222)
            ),
            Commands.print("level4")
            
        );
    }
}


//arm -304  al 2

// ele 26.6   arm -338     al 3

// arm -232 l2

// arm -219   ele 11.1  l3

// arm -229 ele 38  l4

// ele 8.6   l1 