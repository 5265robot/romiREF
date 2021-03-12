// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

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
    public static final class LightInput {
      public static final int leftInput = 0;
      public static final int rightInput = 1;
      public static final double baseSpeed = 1;
      public static final double sensitivity = 0.1;
    }    
    
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
        public static final double kMaxSpeedMetersPerSecond = 0.4; // was 0.8
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.30; // was 0.8
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;
      }

    public static final class TrajectoryConstants {
        // pick either meters or inches to match trajectory values
        private static final double kWheelDiameterMeter = 0.070;
        public static final double kWheelDiameter = kWheelDiameterMeter;
        // private static final double kWheelDiameterInch = 2.75591;
        // public static final double kWheelDiameter = kWheelDiameterInch;
        // STRAIGHT line test 
        public static final List<Translation2d> interiorStraightPoints =
          List.of(
            new Translation2d(Units.inchesToMeters(+16.0), Units.inchesToMeters(+16.0)),
            new Translation2d(Units.inchesToMeters(+22.0), Units.inchesToMeters(+10.0))
          );
        // WAVE in the middle
        public static final List<Translation2d> interiorWavePoints =
          List.of(
            new Translation2d(Units.inchesToMeters(+6.0), Units.inchesToMeters(+0.0)),
            new Translation2d(Units.inchesToMeters(+6.0), Units.inchesToMeters(+6.0)),
            new Translation2d(Units.inchesToMeters(+6.0), Units.inchesToMeters(+12.0))
          );
        // MONDRIAN MADNESS
        public static final List<Translation2d> interiorMondrianPoints =
          List.of(
            //
            // inital pose is Units.inchesToMeters(-13.5),Units.inchesToMeters(4.5)
            // 9 inches down
            // new Translation2d(Units.inchesToMeters(-12.5),Units.inchesToMeters(-4.5)),
            // begin the curve
            new Translation2d(Units.inchesToMeters(-11.4),Units.inchesToMeters(-12.8)),
            // enter the yellow
            // new Translation2d(Units.inchesToMeters(-4.5),Units.inchesToMeters(-13.0)),
            // 9 inches right to be 90 degrees from initial pose
            new Translation2d(Units.inchesToMeters(+4.8),Units.inchesToMeters(-13.0)),
            // align for center traverse
            // new Translation2d(Units.inchesToMeters(+4.5),Units.inchesToMeters(-10.0)),

            // center
            // new Translation2d(Units.inchesToMeters(+0.0),Units.inchesToMeters(+0.0)),
            
            // everything is now a reflection of the original route
            // in reverse order (which means the sign changes, not the value)
            // exit the center
            // new Translation2d(Units.inchesToMeters(-2.0),Units.inchesToMeters(+9.0)),
            // traverse the blue to be a reflection of the 90 degree position
            new Translation2d(Units.inchesToMeters(-1.8),Units.inchesToMeters(+9.8)),
            // 9 inches right
            // new Translation2d(Units.inchesToMeters(+4.5),Units.inchesToMeters(+13.0)),
            // begin the curve
            new Translation2d(Units.inchesToMeters(+11.0),Units.inchesToMeters(+10.6))
            // align for entering the red
            // new Translation2d(Units.inchesToMeters(+14.5),Units.inchesToMeters(+4.5))
            // 9 inches down
            // ending pose is Units.inchesToMeters(+13.5),Units.inchesToMeters(-4.5)
          );
        public static final Pose2d startPose =
          // straight
          // new Pose2d(Units.inchesToMeters(-6.0),Units.inchesToMeters(-6.0), new Rotation2d(Units.degreesToRadians(0)));
          // wave
          // new Pose2d(Units.inchesToMeters(+0.0),Units.inchesToMeters(+0.0),new Rotation2d(Units.degreesToRadians(0.0))); 
          // mondrian
          new Pose2d(Units.inchesToMeters(-13.5), Units.inchesToMeters(+4.5), new Rotation2d(Units.degreesToRadians(-90)));
        public static final Pose2d endPose = 
          // straight
          // new Pose2d(Units.inchesToMeters(-6.0),Units.inchesToMeters(+6.0), new Rotation2d(Units.degreesToRadians(0)));
          //wave
          // new Pose2d(Units.inchesToMeters(+6.0),Units.inchesToMeters(+13.0),new Rotation2d(Units.degreesToRadians(90)));
          // mondrian 
          new Pose2d(Units.inchesToMeters(+11.0), Units.inchesToMeters(-3.1), new Rotation2d(Units.degreesToRadians(-90)));
    }

}
