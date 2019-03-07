/**
 *  Class that organizes gains used when assigning values to slots
 */
package frc.robot;

public class Gains {
	public final double P;
	public final double I;
	public final double D;
	public final double F;
	public final int IZone;
	public final double PeakOutput;
	
	public Gains(double p, double i, double d, double f, int iZone, double peakOutput){
		P = p;
		I = i;
		D = d;
		F = f;
		IZone = iZone;
		PeakOutput = peakOutput;
	}
}