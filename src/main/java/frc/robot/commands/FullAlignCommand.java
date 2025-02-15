package frc.robot.commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.VisionSubsystem;
import pabeles.concurrency.ConcurrencyOps.Reset;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.configs.jni.ConfigJNI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.units.measure.*;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class FullAlignCommand extends ParallelCommandGroup {

    private final CommandSwerveDrivetrain m_Swerve;
    private final Limelight m_limelight;
    private final Pigeon2 m_Pigeon2;
    private Pose2d taPose2d;

    public FullAlignCommand(CommandSwerveDrivetrain swerveDrivetrain, Limelight limelight, Pigeon2 pidgey){
        m_Swerve = swerveDrivetrain;
        m_limelight = limelight;
        m_Pigeon2 = pidgey;
        addCommands(
            new ParallelCommandGroup(
                new ResetGyro(swerveDrivetrain, limelight, pidgey),
                new AlignCommand(swerveDrivetrain, limelight, pidgey)
            )
            
        );
    }

}