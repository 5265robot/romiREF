// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // possibly use with not driving straight?
    public static final class Weirdness {
      public static final double leftTweak = 1.0;
      public static final double rightTweak = 1.0;
    }

    // copied from chief delphi example
    public static final class DriveConstants {
        public static final double ksVolts = 0.929;
        public static final double kvVoltSecondsPerMeter = 6.33;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0389;
    
        public static final double kPDriveVel = 0.085;

        // check this width
        public static final double kTrackwidthMeters = 0.142072613;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
      }
    
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.8;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.8;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
      }

    public static final class TrajectoryConstants {
        // pick either meters or inches to match trajectory values
        private static final double kWheelDiameterMeter = 0.070;
        public static final double kWheelDiameter = kWheelDiameterMeter;
        // private static final double kWheelDiameterInch = 2.75591;
        // public static final double kWheelDiameter = kWheelDiameterInch;
        // for one meter example 
        public static final List<Translation2d> interiorPoints =
          List.of(
            new Translation2d(0.35, 0.0),
            new Translation2d(0.35, 0.45),
            new Translation2d(-0.3, 0.1)
          );
        // for inch example
        public static final List<Translation2d> interiorMondrianPoints =
          List.of(
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d()
          );
        public static final Pose2d startPose = 
          new Pose2d(0, 0, new Rotation2d(0));
        public static final Pose2d endPose = 
          new Pose2d(-0.5, 0.1, new Rotation2d(0));  //(Math.PI)),
    }

    /*public static final class AutoDistConstants {
      public static final List<CommandBase> distance01 =
        List.of(
          new DriveDistance(-0.5, 10, drivetrain),
          new TurnDegrees(-0.5, 180, drivetrain),
          new DriveDistance(-0.5, 10, drivetrain),
          new TurnDegrees(0.5, 180, drivetrain));
    }*/
}
