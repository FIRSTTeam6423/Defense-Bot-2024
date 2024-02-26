package frc.robot;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

   /**
     * DriveUtil Constants
     */
    public static final int FRONTLEFT_DRIVE = 1;
    public static final int FRONTLEFT_PIVOT = 2;
    public static final int FRONTRIGHT_DRIVE = 3;
    public static final int FRONTRIGHT_PIVOT = 4;
    public static final int BACKLEFT_DRIVE = 5;
    public static final int BACKLEFT_PIVOT = 6;
    public static final int BACKRIGHT_DRIVE = 7;
    public static final int BACKRIGHT_PIVOT = 8;
    
    public static final int FRONTLEFT_ABS_ENCODER = 0;
    public static final int FRONTRIGHT_ABS_ENCODER = 1;
    public static final int BACKLEFT_ABS_ENCODER = 2;
    public static final int BACKRIGHT_ABS_ENCODER = 3;

  /**
   * DriveUtil Constants
   */
  //public static final double WHEEL_RADIUS = 0.5;// its 2 inches?????
  public static final double XBOX_STICK_DEADZONE_WIDTH = 0.05;
  public static final double MAX_ANGULAR_SPEED = 720; //
  public static final double MAX_LINEAR_SPEED = 10; //meters per second
  
  //public static final double DRIVEPOSITIONCONVERSIONFACTOR = (1/7.13) * .096 * Math.PI;
  public static final double WHEEL_DIAMETER_INCHES=4;
  public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI; //THIS IS EQUAL TO THE CIRCUMFERENCE OF THE WHEEL
  public static final double DRIVE_GEAR_RATIO=6.55;
  public static final double DRIVE_ROTATIONS_TO_METERS=WHEEL_CIRCUMFERENCE_METERS/DRIVE_GEAR_RATIO;
  public static final double RPM_TO_METERS_PER_SEC = DRIVE_ROTATIONS_TO_METERS/60;//default sparkmax velocity units is RPM so divide by 60

  public static final double MODULEDRIVE_P = 0.01;//.8;//0.03975;
  public static final double MODULEDRIVE_I = 0;
  public static final double MODULEDRIVE_D = 0;
  
  public static final double MODULEPIVOT_P = 0.005;
  public static final double MODULEPIVOT_I = 0;
  public static final double MODULEPIVOT_D = 0;

    public static final double DEGREES_PER_ROTATION = 360;

    public static final double FRONTLEFT_ABS_ENCODER_OFFSET = 317.;
    public static final double FRONTRIGHT_ABS_ENCODER_OFFSET = 246;
    public static final double BACKLEFT_ABS_ENCODER_OFFSET = 236;
    public static final double BACKRIGHT_ABS_ENCODER_OFFSET = 275;

    public static final double[] ABS_ENCODER_OFFSETS = {
        FRONTLEFT_ABS_ENCODER_OFFSET,
        FRONTRIGHT_ABS_ENCODER_OFFSET,
        BACKLEFT_ABS_ENCODER_OFFSET,
        BACKRIGHT_ABS_ENCODER_OFFSET
    };

    //TODO: FIX THIS ITS PROLLY WRONG
    public static final double MODULE_DIST_METERS = Units.inchesToMeters(16.6);
    public static final double FRONTLEFT_X = MODULE_DIST_METERS;//0.224;
    public static final double FRONTLEFT_Y = MODULE_DIST_METERS;//0.224; //swap to negative
    public static final double FRONTLEFT_ANGLE = 45;
    public static final double FRONTRIGHT_X = MODULE_DIST_METERS;//0.224;
    public static final double FRONTRIGHT_Y = -MODULE_DIST_METERS;//-0.224; //swap to positive
    public static final double FRONTRIGHT_ANGLE = 315;
    public static final double BACKLEFT_X = -MODULE_DIST_METERS;//-0.224;
    public static final double BACKLEFT_Y = MODULE_DIST_METERS;//0.224; //swap to negative
    public static final double BACKLEFT_ANGLE = 135;
    public static final double BACKRIGHT_X = -MODULE_DIST_METERS;//-0.224;
    public static final double BACKRIGHT_Y = -MODULE_DIST_METERS;//-0.224; //swap to positve
    public static final double BACKRIGHT_ANGLE = 225;

    /**
     * Controller Input Device Mapping
     * 
     */
    public static final int XBOX_DRIVER = 0;
}
