package frc.robot.commands.reefStuff;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.gripper.ReleaseGripperCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class ScoreL3Command extends SequentialCommandGroup{
    public ScoreL3Command(Elevator elevator, Arm arm, Gripper gripper, Trigger shootTrigger, Trigger confirmTrigger, Trigger confirm2Trigger){
        addCommands(
            new ParallelCommandGroup(

                new MovePCM(elevator, 12),
                new SequentialCommandGroup(
                    new WaitUntilCommand(confirm2Trigger.or(confirmTrigger)),
                    new WaitUntilCommand(elevator.aboveIntake),
                    new MovePCM(arm, -222),
                    new WaitUntilCommand(shootTrigger),
                    new ReleaseGripperCommand(gripper),
                    new WaitCommand(0.5)
                )   
            )
        );  
    }
}
