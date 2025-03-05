package frc.robot.commands.Intake.flop;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeFlop;

public class RetractFlopCommand extends Command {
    private final IntakeFlop intakeFlop;

    public RetractFlopCommand(IntakeFlop intakeFlop) {
        this.intakeFlop = intakeFlop;
        addRequirements(intakeFlop);
    }

    @Override
    public void execute() {
        intakeFlop.setNeutralMode(NeutralModeValue.Brake);
        intakeFlop.goToPosition(0.0);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop intakeFlop when command ends
        intakeFlop.stop();
    }
}