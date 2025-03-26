package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.IntakeFlop;
import frc.robot.subsystems.IntakeRoll;

public class RetractAndHandOff extends SequentialCommandGroup{
    public RetractAndHandOff(Elevator elevator, Arm arm, Gripper gripper, IntakeFlop intakeFlop, IntakeRoll intakeRoll) {
        addCommands(
            new RetractIntakeCommand(intakeFlop, intakeRoll),
            new WaitUntilCommand(intakeFlop.retractedTrigger),
            new CoralHandOff(elevator, arm, gripper)
        );
    }
}
