package frc.robot.commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.VisionSubsystem;

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
import edu.wpi.first.units.measure.*;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class ResetGyro extends Command {
    private final CommandSwerveDrivetrain m_Swerve;
    private final Limelight m_limelight;
    private final Pigeon2 m_Pigeon2;
    

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
   .withDeadband(4.73 * 0.1).withRotationalDeadband(2 * 0.1); // Add a 10% deadband


    public ResetGyro(CommandSwerveDrivetrain swerve, Limelight limelight, Pigeon2 pidgey) {
        m_Swerve = swerve;
        m_limelight = limelight;
        m_Pigeon2 = pidgey;

        addRequirements(swerve, limelight);
    }

    private double limelightAimProportional() {
        // kP (constant of proportionality)
        // Determines the aggressiveness of the proportional control loop
        double kP = 0.06;
    
        // Get the "tx" value from the Limelight
        double targetingAngularVelocity = m_limelight.get_tx() * kP;

        SmartDashboard.putNumber("limelightX: ", m_limelight.get_tx());

    
        // Convert to radians per second for the drivetrain
        targetingAngularVelocity *= 0.75;
    
        // Invert since tx is positive when the target is to the right of the crosshair
        // targetingAngularVelocity *= 1.0;
    
        return targetingAngularVelocity;
    }
    
    // Proportional ranging control with Limelight's "ty" value
    // Works best if the Limelight's mount height and target mount height are different.
    private double limelightRangeProportional() {
        double kP = 0.06;
    
        // Get the "ty" value from the Limelight
        // double targetingForwardSpeed = m_Vision.getTY() * kP;
        double targetingForwardSpeed = m_limelight.get_ty() * kP;


    
        // Convert to meters per second for the drivetrain
        targetingForwardSpeed *= -TunerConstants.kSpeedAt12Volts.magnitude();
    
        // Invert the direction for proper control
        // targetingForwardSpeed *= 1.0;
    
        return targetingForwardSpeed;
    }
    // private double limelightRotProportional() {
    //     double kP = 0.06;
    
    //     // Get the "ty" value from the Limelight
    //     // double targetingForwardSpeed = m_Vision.getTY() * kP;
    //     double targetingForwardSpeed = m_limelight.get_tl() * kP;

    //     //SmartDashboard.putNumber("limelightX", LimelightHelpers.getTY("limelight"));
    
    //     // Convert to meters per second for the drivetrain
    //     targetingForwardSpeed *= TunerConstants.kSpeedAt12Volts.magnitude();
    
    //     // Invert the direction for proper control
    //     // targetingForwardSpeed *= 1.0;
    
    //     return targetingForwardSpeed;
    // }

    private double LimelightRoation(){
        double kP = 0.06;
        double angle = m_Pigeon2.getAccumGyroY().getValueAsDouble();

        return angle;

    }




    public void execute(){

        double tagYaw = LimelightHelpers.getBotPose_wpiBlue("limelight-seaweed")[5];

        m_Swerve.setControl(
            m_driveRequest
                //.withVelocityX(xSpeed * 0.06)
                //.withVelocityY(yspeed)
                .withRotationalRate(-tagYaw * 0.06)
        );
        SmartDashboard.putNumber("Angle",m_Pigeon2.getAccumGyroY().getValueAsDouble());
        //m_Swerve.applyRequest(()->m_driveRequest.withVelocityX(xSpeed));
    }
}