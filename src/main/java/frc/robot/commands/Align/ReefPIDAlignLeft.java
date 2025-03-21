package frc.robot.commands.Align;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ReefPIDAlignLeft extends SequentialCommandGroup {
    public ReefPIDAlignLeft(CommandSwerveDrivetrain drivetrain){
        addCommands(
            new PIDAlign(drivetrain), 
            new PIDFineAlign(true, drivetrain)
        );
    }

    
}
