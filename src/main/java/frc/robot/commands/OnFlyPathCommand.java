package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveSubsystem;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.ctre.phoenix6.configs.jni.ConfigJNI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class OnFlyPathCommand extends Command {
    private final CommandSwerveDrivetrain swerve;

    public OnFlyPathCommand(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {

        double[] transform = Constants.VisionConstants.aprilTagMap.get(18);
        swerve.getPathPlannerCommandToAprilTag(new Pose2d(new Translation2d(transform[3], transform[11]), new Rotation2d()));
    }

    @Override
    public void execute() {

    }
}