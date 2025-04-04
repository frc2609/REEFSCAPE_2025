package frc.robot.commands.reefStuff.Algae;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.gripper.GripCommand;
import frc.robot.commands.gripper.ReleaseGripperCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;


public class AlgaeNetAuto extends SequentialCommandGroup{
    public AlgaeNetAuto(Elevator elevator, Arm arm, Gripper gripper) {
        addCommands(
            new MovePCM(elevator, 39.5),//9
            new WaitUntilCommand(elevator.aboveIntake),
            new MovePCM(arm, 60),
            Commands.runOnce(()->SmartDashboard.putString("state", "elevator -> 39.5 and arm -> 60")),
            new WaitCommand(1),
            new ReleaseGripperCommand(gripper).withTimeout(1),
            Commands.runOnce(()->SmartDashboard.putString("state", "ungripping"))
            

        );
    }
  

}
