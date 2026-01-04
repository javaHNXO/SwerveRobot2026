package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;



  public SwerveSubsystem(File directory)
  {
    boolean blueAlliance = false;
    //Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
    //                                                                  Meter.of(4)),
    //                                                Rotation2d.fromDegrees(0))
    //                                   : new Pose2d(new Translation2d(Meter.of(16),
    //                                                                  Meter.of(4)),
    //                                                Rotation2d.fromDegrees(180));
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    //SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    //try
    //{
    //  swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    //} catch (Exception e)
    //{
    //  throw new RuntimeException(e);
    //}
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
  }



  @Override
  public void periodic() {
  }



  @Override
  public void simulationPeriodic() {
  }



  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }



  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }



  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }



  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }



  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }



  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }



  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }



  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }
}
