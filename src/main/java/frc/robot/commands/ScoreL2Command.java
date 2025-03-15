package frc.robot.commands;

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

public class ScoreL2Command extends SequentialCommandGroup{
    public ScoreL2Command(Elevator elevator, Arm arm, Gripper gripper, Trigger shootTrigger, Trigger confirmTrigger){
        addCommands(
            new WaitUntilCommand(confirmTrigger),
            new ParallelCommandGroup(
            new MovePCM(arm, -228),
            new SequentialCommandGroup(
                new WaitUntilCommand(arm.aboveIntake),
                new MovePCM(elevator, 0.2),
                new WaitUntilCommand(shootTrigger),
                new ReleaseGripperCommand(gripper),
                new WaitCommand(0.5)
           
                 )   
              ) 
        );
    }
}
