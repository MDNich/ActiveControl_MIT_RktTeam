package info.openrocket.core.simulation.ensemble;

/** Deterministic continuous Gaussian process with independent knots. No RNG draws in the solver.
 * Variance-normalized interpolation preserves the requested marginal sigma between knots.
 * Correlation goes to zero at two knot intervals; this is sampled noise, not mathematical white noise.
 */
public final class GaussianThrustNoise {
    private final long seed;
    private final double sigma, interval;
    public GaussianThrustNoise(long seed, double sigma, double interval) {
        if (!Double.isFinite(sigma) || sigma < 0 || !Double.isFinite(interval) || interval <= 0)
            throw new IllegalArgumentException("Invalid thrust noise");
        this.seed=seed; this.sigma=sigma; this.interval=interval;
    }
    public double value(double time, long motorIdentity) {
        if (sigma == 0 || !Double.isFinite(time) || time < 0) return 0;
        double at=time/interval; long k=(long)Math.floor(at); double f=at-k;
        long key=mix(seed ^ mix(motorIdentity));
        return sigma*((1-f)*gaussian(key,k)+f*gaussian(key,k+1))/Math.hypot(1-f,f);
    }
    private static double gaussian(long key, long k) {
        double u=unit(mix(key+2*k)), v=unit(mix(key+2*k+1));
        return Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*v);
    }
    private static double unit(long x) { return ((x >>> 11)+.5)*0x1.0p-53; }
    private static long mix(long z) {
        z=(z^(z>>>30))*0xbf58476d1ce4e5b9L; z=(z^(z>>>27))*0x94d049bb133111ebL; return z^(z>>>31);
    }
}
