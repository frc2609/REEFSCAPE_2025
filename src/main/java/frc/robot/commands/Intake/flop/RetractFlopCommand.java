package frc.robot.commands.Intake.flop;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pcmUtils.MovePCM;
import frc.robot.subsystems.IntakeFlop;

public class RetractFlopCommand extends SequentialCommandGroup {
    private final double target = 0;

    public RetractFlopCommand(IntakeFlop intakeFlop) {
        addRequirements(intakeFlop);
        addCommands(
            new InstantCommand(() -> intakeFlop.setNeutralMode(NeutralModeValue.Brake)),
            new MovePCM(intakeFlop, target)
        );
    }



}