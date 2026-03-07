package frc.robot.subsystems.climb;

public final class ClimbConstants {
	private ClimbConstants() {
	}

	// CAN ID layout (non-drive devices)
	public static final int CLIMB_MOTOR_ID = 59;

	public static final double MOTOR_ROTATIONS_PER_OUTPUT_REV = 81.0;
	public static final double OUTPUT_REVOLUTIONS_PER_COMMAND = 1.25;
	public static final double MOTOR_ROTATIONS_PER_COMMAND =
			MOTOR_ROTATIONS_PER_OUTPUT_REV * OUTPUT_REVOLUTIONS_PER_COMMAND;
	public static final double POSITION_TOLERANCE_ROTATIONS = 0.35;

	public static final double MAX_DUTY_CYCLE = 0.60;
	public static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
	public static final double STATOR_CURRENT_LIMIT_AMPS = 80.0;
	public static final double CLOSED_LOOP_RAMP_SECONDS = 0.2;
	public static final double SLOT0_KP = 0.8;
	public static final double SLOT0_KD = 0.0;

	public static final double SIM_RESPONSE_ALPHA = 0.20;
	public static final double SIM_MAX_RPS = 120.0;
}
