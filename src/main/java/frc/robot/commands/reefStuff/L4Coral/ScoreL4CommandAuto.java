package frc.robot.commands.reefStuff.L4Coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Align.PIDFineAlign;
import frc.robot.commands.gripper.ReleaseGripperCommand;
import frc.robot.commands.gripper.SlowGripCommand;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class ScoreL4CommandAuto extends SequentialCommandGroup{
    public ScoreL4CommandAuto(Elevator elevator, Arm arm, Gripper gripper){
        addCommands(
         new MovePCM(elevator, 8.5),
         new MovePCM(arm, 0),
         new SlowGripCommand(gripper),
         Commands.runOnce(()->SmartDashboard.putString("state", "elevator -> 8.5 and arm -> 0")),
        new ParallelCommandGroup(
            new MovePCM(elevator, 39), //39
            Commands.runOnce(()->SmartDashboard.putString("state", "elevator -> 39 and arm -> 0")),
            new SequentialCommandGroup(
                new WaitUntilCommand(elevator.aboveIntake),
                //new WaitUntilCommand(PIDFineAlign.aligned).withTimeout(2),   This one is not gonna be used anymore in auto cuz we are not using PID align
                new MovePCM(arm, -230), //-227
                Commands.runOnce(()->SmartDashboard.putString("state", "elevator -> 39 and arm -> -230")),
                new WaitUntilCommand(() -> elevator.getPosition() > 37 && arm.getPosition() < -226),
                new ReleaseGripperCommand(gripper).withTimeout(0.75),
                Commands.runOnce(()->SmartDashboard.putString("state", "ungripping")),
                new WaitCommand(0.75)

            )
            )
        );
    }
}