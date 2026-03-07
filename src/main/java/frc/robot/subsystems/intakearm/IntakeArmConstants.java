package frc.robot.subsystems.intakearm;

public final class IntakeArmConstants {
	private IntakeArmConstants() {
	}

	public static final int ARM_MOTOR_ID = 57;
	public static final int ARM_CANCODER_ID = 58;

	// Measured CANcoder absolute position when arm is fully up (reference zero).
	public static final double ARM_UP_REFERENCE_ROT = -0.279785;
	// Keep this aligned with what you measured in Tuner X.
	public static final boolean ARM_CANCODER_CCW_POSITIVE = true;

	public static final double ARM_UP_ANGLE_DEG = 0.0;
	// Measured travel from up (-0.279785 rot) to down (-0.491943 rot): -76.37688 deg
	public static final double ARM_DOWN_ANGLE_DEG = -76.38;

	// Keep this sign consistent with the measured up/down absolute rotations above.
	public static final double ARM_DIRECTION = 1.0;

	
	public static final double ARM_KP = 0.03;
	public static final double ARM_MAX_OUTPUT = 0.1;
	// Minimum output to overcome static friction when moving.
	public static final double ARM_MIN_MOVING_OUTPUT = 0.04;
	public static final double ARM_ANGLE_TOLERANCE_DEG = 0.25;

	public static final double ARM_MOTOR_SUPPLY_CURRENT_LIMIT_A = 35.0;
	public static final double ARM_MOTOR_STATOR_CURRENT_LIMIT_A = 70.0;
	public static final double ARM_JOG_STEP_DEG = 1.0;
}
