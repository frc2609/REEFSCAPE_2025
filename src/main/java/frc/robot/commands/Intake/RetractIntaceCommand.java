package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Intake.flop.RetractFlopCommand;
import frc.robot.commands.Intake.roll.StopRollCommand;
import frc.robot.subsystems.IntakeFlop;
import frc.robot.subsystems.IntakeRoll;

public class RetractIntaceCommand extends ParallelCommandGroup{
    public RetractIntaceCommand(IntakeFlop intakeFlop, IntakeRoll intakeRoll) {
        addCommands(
            new RetractFlopCommand(intakeFlop),
            new StopRollCommand(intakeRoll)
        );
    }
    
}
