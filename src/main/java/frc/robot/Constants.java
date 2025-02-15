// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import java.util.HashMap;

import edu.wpi.first.units.measure.*; 

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Swerve {
    public static final Translation2d flModuleOffset = new Translation2d(0.546 / 2.0, 0.546 / 2.0);
    public static final Translation2d frModuleOffset = new Translation2d(0.546 / 2.0, -0.546 / 2.0);
    public static final Translation2d blModuleOffset = new Translation2d(-0.546 / 2.0, 0.546 / 2.0);
    public static final Translation2d brModuleOffset = new Translation2d(-0.546 / 2.0, -0.546 / 2.0);

    public static final double maxModuleSpeed = 4.5; // M/S

    public static final PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants rotationConstants = new PIDConstants(5.0, 0.0, 0.0);




  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

    public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "limelight";
    public static final Distance LIMELIGHT_LENS_HEIGHT = Distance.ofBaseUnits(31, Inches);
    public static final Angle LIMELIGHT_ANGLE = Angle.ofBaseUnits(45, Degrees);

    public static final Distance REEF_APRILTAG_HEIGHT = Distance.ofBaseUnits(6.875, Inches);
    public static final Distance PROCCESSOR_APRILTAG_HEIGHT = Distance.ofBaseUnits(45.875, Inches);
    public static final Distance CORAL_APRILTAG_HEIGHT = Distance.ofBaseUnits(53.25, Inches);

    public static final HashMap<Integer, double[]> aprilTagMap;
    static {
      aprilTagMap = new HashMap<>();
      aprilTagMap.put(17, new double[] {
        -0.5000000000000002,
        0.8660254037844384,
        0,
        -4.700094,
        -0.8660254037844384,
        -0.5000000000000002,
        0,
        -0.7196820000000002,
        0,
        0,
        1,
        0.308102,
        0,
        0,
        0,
        1
      });
      aprilTagMap.put(18, new double[] {
        1,
        1.2246467991473532e-16,
        0,
        5.116399999999999,
        1.2246467991473532e-16,
        1,
        0,
        0.00009999999999976694,
        0,
        0,
        1,
        0.308102,
        0,
        0,
        0,
        1
      });
    }
  }

}
}