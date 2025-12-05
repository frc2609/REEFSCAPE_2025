package frc.robot.commands.reefStuff.Algae;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.gripper.GripCommand;
import frc.robot.commands.gripper.ReleaseGripperCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;


public class PickAlgaeL2CommandAuto extends SequentialCommandGroup{
    public PickAlgaeL2CommandAuto(Elevator elevator, Arm arm, Gripper gripper) {
        addCommands(
            new MovePCM(elevator, 10),//9
            new ParallelCommandGroup(
                new GripCommand(gripper),
                new MovePCM(arm, 40),
                Commands.runOnce(()->SmartDashboard.putString("state", "elevator -> 10 and arm -> 40 and algae in"))
  
            ).withTimeout(5)


        );
    }
  

}
