# Supersonic finite-wing experiment (MIT 6.6)

In a flight simulation's **Simulation options**, enable **Experimental supersonic trapezoidal fins**. The choice is per simulation, defaults to legacy, and is saved in the .ork. FinModelBenchmark invokes the same Barrowman calculator as the flight integrator. The ordinary design-display CP and Component Analysis use their existing model selection.

## Pressure audit and scope

At zero incidence the legacy supersonic single-fin slope is the exposed area times 2/β, divided by the vehicle reference area, where β = √(Mach² − 1). Its origin is the one-surface pressure expansion in [OpenRocket technical documentation, eqs. 3.41–3.49](https://openrocket.sourceforge.net/techdoc.pdf#page=39). The two-face, infinite-span, thin-plate limit at small incidence is 4/β when referenced to one projected panel area. Applying that limit to an entire real fin would omit finite-span relief.

The candidate integrates the two-face load and its axial first moment over a trapezoid. It applies the linear rectangular-wing tip-cone pressure distribution downstream of the tip leading edge as a **swept-planform approximation**. Inside the cone the local fraction of the 2-D pressure jump is (2/π) asin(√(β y/x)), with y measured inboard from the tip and x downstream from the tip leading edge. Outside the cone it is one. The mirrored fin root and the existing Barrowman body/fin multiplier approximate body interference. The existing instance orientation, fin-count interference and cant calculations apply afterwards. Existing drag, roll damping and body aerodynamics are used in both runs. There is no section-thickness, shock/body interaction, or full three-dimensional pressure solution in this experiment. [Rectangular finite-wing derivation](https://farside.ph.utexas.edu/teaching/336L/Fluidhtml/node220.html)

The quadrature uses 48 spanwise strips and 32 chordwise midpoints. Per-geometry force and first-moment integrals are tabulated every 0.025 Mach from 1.1 to 5. Cubic smoothstep weights blend both force and first moment with legacy, after which their ratio gives the load position. The Mach handover spans 1.1–1.5 and ends at 4.65–5. The leading-edge-normal Mach handover spans 1–1.15. The model also requires β times panel span to exceed the aft length downstream of the tip leading edge, with the handover covering 0.15 span of clearance. The full incidence range is 0–3°, fading to legacy at 8°. These handover widths are engineering choices. Other planforms retain their existing calculation.

## Wind-tunnel benchmarks

The geometry is in FinModelBenchmark.java, and measurements.csv contains approximate graph readings. CNa is per radian, CP is aft of the nose in body diameters. The pitching-moment slope for comparison is CNa times (reference station minus CP) in calibers. It is reconstructed from the measured pair and is correlated with both measurements.

- **ARCAS**, half-scale configuration 1, zero cant. [NASA TN D-4014, Fig. 1](https://ntrs.nasa.gov/api/citations/19670020031/downloads/19670020031.pdf#page=10) gives the 40.95 in vehicle, 2.25 in body diameter, 10.60 in ogive, and fin and tail stations. The fixture uses a straight conical boattail, planar trapezoids at the cylindrical mounting radius and omits the small anchor details, nose bluntness and reflexed lip. The moment reference station is 25.95 in. Both observables through M2.30 and CNa through M3.96 are read from the experimental series in [Fig. 6.7](https://openrocket.sourceforge.net/techdoc.pdf#page=103). Above M2.30 the CP values are read from the configuration 1 line in [NASA Fig. 7](https://ntrs.nasa.gov/api/citations/19670020031/downloads/19670020031.pdf#page=43). The historical simulated line is not used as ground truth.
- **Yuska model B**, four swept wedge fins, 0.8° cant, 61.7 in vehicle, 6 in reference diameter. Dimensions follow [NASA TN D-3182, Fig. 1 and model description](https://ntrs.nasa.gov/api/citations/19660004864/downloads/19660004864.pdf#page=9). The fixture approximates the tabulated nose by a three-quarter power profile. Both coefficient series are the model B experimental circles in [Fig. 4(b)](https://ntrs.nasa.gov/api/citations/19660004864/downloads/19660004864.pdf#page=14). Its moment reference is the model base. The thick wedge is a challenging independent geometry for the thin-panel approximation.

The numerical precision of the result file exceeds the precision of reading the graphs. Errors below are unweighted RMS over five supersonic points per configuration, M1.50–3.96 for ARCAS and M2.00–3.47 for Yuska B. The six additional ARCAS samples through M1.20 exercise the low-speed and transonic behavior.

| Supersonic RMS error | ARCAS legacy | ARCAS candidate | Yuska B legacy | Yuska B candidate |
| --- | ---: | ---: | ---: | ---: |
| Normal-force slope, per radian | 5.67 | 2.88 | 4.68 | 2.23 |
| CP, body diameters | 2.66 | 0.32 | 1.02 | 0.13 |
| Pitching-moment slope, per radian | 18.58 | 2.65 | 6.14 | 4.85 |

The candidate still underpredicts CNa at all ten supersonic points. On Yuska B at M3.47 it predicts 6.21/rad against approximately 9.12/rad. On the six low-speed ARCAS points the two models agree and share a CNa RMS error of 4.36/rad. The comparisons support keeping the candidate available for sensitivity studies. They do not establish a universal correction or an uncertainty interval for another rocket.

## Reproduce

From the repository root with JDK 17, build the current MIT shadow JAR. Run the following in bash. CompareFlights takes the operator's .ork, a separate output directory, and the associated RASP motor files. It creates a two-model setup, reopens it, fixes both solver and wind seeds to 20261108, runs both flights, and exports the primary branch history for each.

```sh
jar=clone/openrocket/build/libs/OpenRocket-MIT-v6.6.jar
src=sim/verification/finite-wing
out=build/finite-wing-benchmark
mkdir -p "$out/classes"
"$JAVA_HOME/bin/javac" -encoding UTF-8 -cp "$jar" -d "$out/classes" "$src"/*.java
"$JAVA_HOME/bin/java" -Djava.awt.headless=true -cp "$jar:$out/classes" FinModelBenchmark "$src/measurements.csv" "$out/bench.csv"
"$JAVA_HOME/bin/java" -Djava.awt.headless=true -cp "$jar:$out/classes" CompareFlights "$rocket_file" "$out/flight" "$booster_curve" "$sustainer_curve"
```

FiniteWingPressureTest checks the analytic finite rectangular-panel force and moment, the wide-panel limit, and the incidence, tip-cone and leading-edge envelopes. The benchmark sweeps each vehicle and model at 0.001 Mach from 0.75 through 5.001 and rejects nonfinite loads or large adjacent jumps. The per-point reference output is in [benchmark-results.csv](benchmark-results.csv).
