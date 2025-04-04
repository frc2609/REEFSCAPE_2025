package frc.robot.commands.reefStuff;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.gripper.ReleaseGripperCommand;
import frc.robot.commands.gripper.ReleaseGripperCommandL1;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class ScoreL1Command extends SequentialCommandGroup{
    public ScoreL1Command(Elevator elevator, Arm arm, Gripper gripper, Trigger shootTrigger, Trigger confirmTrigger){
        addCommands(
            new MovePCM(elevator, 10.3),
            //new WaitUntilCommand(confirm2Trigger.or(confirmTrigger)),
            new ParallelCommandGroup(
                new MovePCM(arm, -60),
            new SequentialCommandGroup(
                new WaitUntilCommand(arm.aboveIntake),
                new MovePCM(elevator, 4.14),
                new WaitUntilCommand(shootTrigger),
                new ReleaseGripperCommandL1(gripper),
                new WaitCommand(0.65)
           
                 )   
              ) 
        );
    }
}
