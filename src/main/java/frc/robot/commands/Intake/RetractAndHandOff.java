package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
            // new PrintCommand("In RetractAndHandOff"),
            new RetractIntakeCommand(intakeFlop, intakeRoll),
            // new PrintCommand("After retract, waiting on retracted trigger"),
            new WaitUntilCommand(intakeFlop.retractedTrigger),
            // new PrintCommand("After retracted trigger"),
            new CoralHandOff(elevator, arm, gripper)
        );
    }
}
