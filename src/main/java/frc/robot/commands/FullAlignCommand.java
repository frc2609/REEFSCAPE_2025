package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class FullAlignCommand extends ParallelCommandGroup {


    public FullAlignCommand(CommandSwerveDrivetrain swerveDrivetrain, Limelight limelight, Pigeon2 pidgey){
        addCommands(
            new ParallelCommandGroup(
                new ResetGyro(swerveDrivetrain, limelight, pidgey),
                new AlignCommand(swerveDrivetrain, limelight, pidgey)
            )
            
        );
    }

}