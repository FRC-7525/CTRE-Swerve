package frc.robot.pioneersLib.controlConstants;

public class FFConstants {

	public double kS;
	public double kV;
	public double kA;
	public double kG;

	public FFConstants(double ks, double kg, double kv, double ka) {
		this.kS = ks;
		this.kV = kv;
		this.kA = ka;
		this.kG = kg;
	}

	public FFConstants(double ks, double kv, double ka) {
		this(ks, 0, kv, ka);
	}

	public FFConstants(double ks, double kv) {
		this(ks, 0, kv, 0);
	}

	public FFConstants(double kv) {
		this(0, 0, kv, 0);
	}
}
