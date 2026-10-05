# Ensemble simulation

Implemented in the existing simulation editor and plot tools. An ensemble is one simulation entry containing the mean flight, per-variable standard deviations, individual flight-summary samples, and the full recorded results of all N runs. The mode is off by default.

## Use

1. Edit a simulation and open **Simulation options**. Find **Ensemble simulation** in the left column, above the flight-computer box.
2. Enable **Run multiple flights with variation** and choose the number of runs (2–10,000).
3. Set **Thrust noise σ (N)** and **Noise sample interval (s)**. These add time-varying noise to the thrust curve, not a scale factor.
4. Set the launch-temperature standard deviation in kelvin, pressure standard deviation in percent, and wind-component standard deviation in m/s. A temperature difference of 1 K equals 1 °C.
5. Set unwanted noise sources to zero. The plot identifies motor-only, atmosphere-only, combined, or no added variation. Use a saved random seed to repeat the batch.
6. Run the simulation normally. The progress window shows the current run and total. Console lines beginning `ENSEMBLE` record the settings, per-run atmosphere, motor noise seed, results, and any flight-computer output paths.
7. In **Plot data**, use the usual 2D plot configuration for the average and its ±1σ shading. Choose **3D trajectory** for the mean trajectory, or **Outcome probability density** for flight-summary distributions. The distribution window has quantity, stage, and bin-count selectors. Plot image saving remains available through the chart menu.
8. Save the `.ork` with **all simulated data** enabled to retain every individual flight as well as the mean, bands and distributions. New ensembles default to saving simulation data unless a storage choice was explicitly set previously. Saving without simulation data retains the ensemble settings, but requires another run before plotting.

## Motor and atmosphere models

For each physical motor instance during its burn:

`simulated thrust(t) = max(0, nominal thrust(t) + e(t))`

`e(t)` is a zero-mean Gaussian process with the selected standard deviation **in newtons**. Independent Gaussian values are generated at fixed intervals measured from motor ignition. Between adjacent knots, interpolation is divided by `sqrt((1-f)^2 + f^2)` so that the requested marginal variance also holds between the knots. Nearby times are correlated; samples separated by two knot intervals have no shared knots. This is sampled continuous noise, not ideal white noise. The noise trace is deterministic for the seed, run, motor identity and motor time, so repeated solver evaluations never draw new noise. RK4 and RK6 steps during motor burn are limited to at most one quarter of the selected noise interval, subject to the existing smaller limits.

No noisy thrust is added before ignition or after burnout. Burn timing and propellant mass follow the existing motor model. Clipping at zero makes the actual thrust distribution non-Gaussian near zero nominal thrust and raises its mean there; the entered σ describes the additive noise before clipping. Independent motor instances have independent noise. The motor database itself is unchanged.

Each run independently draws one launch-temperature offset, one launch-pressure offset and two horizontal wind offsets (East and North). Temperature and pressure feed OpenRocket's existing extended ISA model. The wind offsets apply throughout the flight, including multi-level wind profiles. The configured launch rod direction stays fixed. Existing turbulent wind and the solver's baseline seed stay fixed across the batch so their random realization does not add an unselected uncertainty source. Impossible temperature or pressure draws stop the batch with an explanatory error.

Both noise sources can be active together. In that case the displayed spread is their combined result, not a decomposition of their separate contributions. Disable one source to isolate the other; keep the same seed for a paired comparison.

## Meaning of the plots

- Runs are interpolated at the same physical times, using the first run's time grid. Each stage is aggregated separately. Mean trajectories stop at the earliest end/ground-impact time shared by every run; landed flights are not extrapolated or frozen into the mean. This also bounds the 3D trajectory.
- Each displayed mean and deviation requires finite values from all runs at that time. Missing values leave gaps. Standard deviations use the sample divisor `N−1`; this is flight-to-flight spread, **not** a confidence interval for the mean. ±1σ need not contain 68% of a non-Gaussian output distribution.
- Non-time X axes retain time order. Their band shows Y spread at the common time, against the mean X value; it is not a joint X–Y confidence region.
- Wind/position direction and azimuth use circular means and circular deviations. Quaternion samples use hemisphere alignment; the 3D reader normalizes the mean quaternion. These provide a representative orientation, not a dynamically simulated flight of their own.
- Phase markers use the mean event times for matching events present in every run and inside the common plotting interval. Recovery and exhaust animation use those representative times.
- Scalar distributions are computed from the original full run before trajectory averaging. They include maximum altitude, velocity, acceleration before recovery, Mach number, minimum stability over the entire flight, time to apogee, flight duration, rod exit/deployment/impact velocities, optimum delay and final horizontal distance. The simulation table likewise reports the mean of each run's summary, which may differ from the extremum of the mean trajectory.
- A probability-density plot is an empirical histogram normalized to unit total area; outputs are not assumed Gaussian. Undefined outcomes are excluded with an explicit count. Identical outcomes show a point mass instead of an invented finite-width distribution. Configured maximum simulation duration still applies to each run; results beyond that duration are not inferred.

## Execution and persistence

Runs execute sequentially within an ensemble. Each completed flight is retained in a compressed temporary archive while statistics accumulate, keeping only one full flight history in memory at a time. Saving embeds all of these flights in the `.ork` file; the saved file is self-contained and does not need the temporary archive. Reopening rebuilds the compressed archive one flight at a time. Temporary storage is removed when no result references it or when the application exits. Additional simulation listeners and extension configurations are copied for each run. The previous result is replaced only when the entire batch succeeds. A cancelled, aborted, or failed run stops the batch; unsuccessful flights are never silently discarded from the statistics.

Flight-computer CSV/log filenames selected by the user receive a unique batch/run suffix. Empty paths retain the existing unique-directory behavior. Supporting packet and metadata filenames follow the suffixed CSV. Console output lists the per-run paths; there is no single FC telemetry stream for an averaged flight.

Ensemble settings and the settings used by the recorded result are stored separately, so changing options does not relabel old results. The `.ork` stores mean trajectories, deviation trajectories at full numeric precision, per-run scalar samples, and numbered individual runs. Each run includes every recorded variable and stage at its original timestamps, including samples after the common mean-trajectory cutoff, its flight events and warnings, and its sampled motor seed, launch temperature/pressure and East/North wind offsets. No trajectory resampling or averaging is applied to the retained individual results. Full runs stay attached to the ensemble rather than creating N extra simulation entries. Files saved before this addition remain readable but contain only aggregate data; rerun those ensembles to obtain the full histories. Ordinary non-ensemble simulations retain their existing behavior and file precision.

## Verification

Targeted tests exercise Gaussian marginal variance at knots and between them, deterministic evaluation in different orders, noise gating at ignition/burnout, physical-time interpolation, sample deviations, circular directions, quaternion sign alignment, full-run extrema, missing-data gaps, complete simulated flights, repeatability, zero-noise spread, atmosphere-only spread, cancellation/failure retention, file round trips, chart unit conversions, both plot axes, nonmonotone X axes, the mean 3D path, PDF normalization/degenerate data, input validation and editor layout. Rendered verification plots are in `swing/build/ensemble-verification/`.

Validation on 2026-09-27: 30 core and 12 Swing tests passed across the relevant checks, including the running progress dialog. The root `shadowJar` build passed; the packaged startup class, ensemble classes, version 6.3 and checksum were verified. The updated local artifact is `build/libs/OpenRocket-MIT-v6.3.jar`. No release was published for this feature.

Full-flight persistence verification (2026-09-27): round-trip tests compare every recorded value and event timestamp across all runs, preserve sampled inputs, stage branches, per-flight warning references and late trajectory samples, and verify repeated saving after reopening. Tests cover the normal compressed `.ork` container, older aggregate-only files and explicitly saving without simulation data.
