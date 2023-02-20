package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Autos;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
        Autos auto;
        // 4 modules on the drivetrain
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;
        private final Field2d field2d = new Field2d();

        // Contains the current distances and angles of each module
        private final SwerveModulePosition[] modulePositions = { new SwerveModulePosition(), new SwerveModulePosition(),
                        new SwerveModulePosition(), new SwerveModulePosition() };

        // Just an array to reference each module
        private final SwerveModule[] modules = new SwerveModule[4];

        // Gyroscope
        private final AHRS m_navx = new AHRS(Port.kMXP);

        // Used to keep a virtual position of the robot
        private final SwerveDriveOdometry odometry;

        // Contains our speeds
        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public Drivetrain() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
                // The module has two NEOs on it. One for steering and one for driving.
                m_frontLeftModule = Mk3SwerveModuleHelper.createNeo(
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                                DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET);
                m_frontRightModule = Mk3SwerveModuleHelper.createNeo(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);
                m_backLeftModule = Mk3SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                                DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET);
                m_backRightModule = Mk3SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                                DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

                // Adding modules to the array
                modules[0] = m_frontLeftModule;
                modules[1] = m_frontRightModule;
                modules[2] = m_backLeftModule;
                modules[3] = m_backRightModule;

                shuffleboardInit();

                // setting current gyro reading to 0 (accounts for uneven floor)
                zeroGyroscope();

                // Odometry is instantiated intitally with our module positions on the chasis,
                // Rotation (should be 0)
                // and the current positions, each should be 0 still here
                odometry = new SwerveDriveOdometry(DrivetrainConstants.KINEMATICS,
                                m_navx.getRotation2d(), modulePositions);

        }

        // Resetting
        public void zeroGyroscope() {
                m_navx.zeroYaw();
        }

        public void resetPose(Pose2d pose) {
                odometry.resetPosition(getGyroscopeRotation(), modulePositions, pose);
        }

        // Gets
        public Rotation2d getGyroscopeRotation() {
                return m_navx.getRotation2d();
        }

        public Pose2d getPose() {
                return odometry.getPoseMeters();
        }

        // We set the speeds we want from joystick values
        private void setSpeeds(ChassisSpeeds chassisSpeeds) {
                SwerveModuleState[] states = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
                // Update virtual states
                setStates(states);
        }

        public void lockWheels()
        {
                m_frontLeftModule.setDriveMotorIdleState(CANSparkMax.IdleMode.kBrake);
                m_frontRightModule.setDriveMotorIdleState(CANSparkMax.IdleMode.kBrake);
                m_backLeftModule.setDriveMotorIdleState(CANSparkMax.IdleMode.kBrake);
                m_frontRightModule.setDriveMotorIdleState(CANSparkMax.IdleMode.kBrake);
        }

        public void unlockWheels()
        {
                m_frontLeftModule.setDriveMotorIdleState(CANSparkMax.IdleMode.kCoast);
                m_frontRightModule.setDriveMotorIdleState(CANSparkMax.IdleMode.kCoast);
                m_backLeftModule.setDriveMotorIdleState(CANSparkMax.IdleMode.kCoast);
                m_frontRightModule.setDriveMotorIdleState(CANSparkMax.IdleMode.kCoast);
        }

        // Setting each module to be that speed
        public void setStates(SwerveModuleState[] state) {
                odometry.update(getGyroscopeRotation(), modulePositions);

                m_frontLeftModule.set(
                                (state[0].speedMetersPerSecond / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                                                * DrivetrainConstants.MAX_VOLTAGE)
                                                * DrivetrainConstants.SPEED_LIMIT,
                                state[0].angle.getRadians());
                m_frontRightModule.set(
                                (state[1].speedMetersPerSecond / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                                                * DrivetrainConstants.MAX_VOLTAGE)
                                                * DrivetrainConstants.SPEED_LIMIT,
                                state[1].angle.getRadians());
                m_backLeftModule.set(
                                (state[2].speedMetersPerSecond / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                                                * DrivetrainConstants.MAX_VOLTAGE)
                                                * DrivetrainConstants.SPEED_LIMIT,
                                state[2].angle.getRadians());
                m_backRightModule.set(
                                (state[3].speedMetersPerSecond / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
                                                * DrivetrainConstants.MAX_VOLTAGE)
                                                * DrivetrainConstants.SPEED_LIMIT,
                                state[3].angle.getRadians());
        }

        // Positions
        public SwerveModulePosition getPosition(int moduleNumber) {
                return new SwerveModulePosition(
                                (modules[moduleNumber].getDriveEncoder().getPosition() *
                                                (DrivetrainConstants.WHEEL_DIAMETER
                                                                * Math.PI / (DrivetrainConstants.GEAR_RATIO * 2048.0))),
                                new Rotation2d(modules[moduleNumber].getSteerAngle()));
        }

        private void updatePositions() {
                modulePositions[0] = getPosition(0);
                modulePositions[1] = getPosition(1);
                modulePositions[2] = getPosition(2);
                modulePositions[3] = getPosition(3);
        }

        // needed for the drive command
        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }

        public void shuffleboardInit() {
                ShuffleboardTab drivetrain = Shuffleboard.getTab("Drivetrain");
                drivetrain.add("Field", field2d);
        }

        @Override
        public void periodic() {
                // setting the speeds of each module
                setSpeeds(this.m_chassisSpeeds);
                // updating the positions of each module
                updatePositions();
        }
}
