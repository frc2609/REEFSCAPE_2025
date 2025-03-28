package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.gripper.GripCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class HumanIntakeCommand extends SequentialCommandGroup {
    public HumanIntakeCommand(Arm arm, Gripper gripper, Elevator elevator){
        addCommands(
            new MovePCM (elevator, 9),
            new GripCommand(gripper),
            new WaitUntilCommand(elevator.aboveIntake),
            new MovePCM(arm,-70 ),//-91
            new MovePCM (elevator,14 ),//6
            new WaitCommand(100)
        );
    }
}
