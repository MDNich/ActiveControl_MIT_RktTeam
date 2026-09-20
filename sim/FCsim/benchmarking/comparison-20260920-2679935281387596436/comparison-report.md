# OpenRocket versus GS1 flight telemetry

The supplied OpenRocket run underpredicts the **barometric height gain by 940 m (17.4%)**, reaches barometric apogee about **0.93 s earlier**, and descends substantially faster. The discrepancy is already large before the airbrakes open. The 3D comparison also shows much less horizontal displacement in the simulation.

As requested, **barometric altitude is the primary altitude reference**. GPS altitude has very low confidence and is used only in a separate diagnostic. The 3D plots use GPS latitude/longitude and barometric height.

## Plots

![Altitude, velocity and airbrake comparison](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/benchmarking/comparison-20260920-2679935281387596436/trajectory-comparison.png)

![3D trajectories and horizontal projection](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/benchmarking/comparison-20260920-2679935281387596436/trajectories-3d.png)

The corresponding SVG files provide vector exports. [Sensor diagnostics](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/benchmarking/comparison-20260920-2679935281387596436/sensor-diagnostics.png) show the altitude-channel disagreement, velocity-estimator drift, boost accelerometer profile and temperature conventions.

## Alignment and reference height

Time zero is FC flight detection: GS1's first `state.FLIGHT` packet at boot **747766 ms**, and the simulation's exact logged transition at boot **1010 ms**. GS1's last preflight packet is 60 ms earlier, so alignment has roughly one packet interval of uncertainty. No curve-fitting shift, time scaling or altitude scaling was applied. Firmware `flight_time` is used instead of the ground station's packet-arrival `timestamp`.

For comparable height gains, each barometer is zeroed to the median of preflight readings in the interval −3 to −0.2 s. GS1 has 48 such samples and a **145.261 m** baseline. The simulation has five samples after its shorter warmup and a **0 m** baseline. This subtraction defines height above the prelaunch sensor reference; it does not use GPS altitude or claim a surveyed elevation. The original values are retained in the table below.

| Quantity | OpenRocket | GS1 | Simulation minus GS1 |
| --- | ---: | ---: | ---: |
| Reported barometer maximum | 4463.704 m | 5549.228 m | −1085.524 m |
| Maximum height gain above prelaunch barometer | 4463.704 m | 5403.967 m | **−940.263 m (−17.4%)** |
| Time of barometer maximum | 27.910 s | 28.836 s | −0.926 s |
| Peak FC integrated velocity | 409.159 m/s | 437.740 m/s | −28.581 m/s |
| Time of peak FC integrated velocity | 2.050 s | 1.812 s | +0.238 s |
| First received open-airbrake command | 14.050 s | 14.040 s | +0.010 s |
| Airbrake closure in telemetry | 27.790 s | 28.956 s | −1.166 s |
| Minimum valid airbrake pulse | 670 µs | 525 µs | +145 µs, less simulated opening |
| First descending sample below 1000 m height gain | 107.530 s | 135.426 s | −27.896 s |
| FC `MAIN` transition | 120.650 s, from log | 149.292 s, first packet | −28.642 s |

The velocity column is the FC's integrated accelerometer estimate, not an independent measurement of vertical speed. Packet-derived event times have their telemetry sampling uncertainty. GS1's `POST_APOGEE` and the Java CSV's `APOGEE` represent state ID 3; this naming difference is a decoder convention.

## What differs and what it suggests

**1. The ascent mismatch precedes braking.** At 10 s, GS1 has gained **3313 m** according to the barometer and the simulation **2734 m**, a **580 m** deficit. At 14 s, just before opening, the deficit is **682 m**. Over 0–28 s, the barometric trajectory error has a −623 m mean and 656 m RMS on the common 0.1 s comparison grid. A 60 ms timing uncertainty cannot explain this magnitude.

The boost accelerometer shapes also differ. The transmitted X channel peaks at **354.7 m/s² at 0.702 s** in GS1, compared with **235.0 m/s² at 1.09 s** in the simulation. GS1 has a rounded early peak; the simulated trace follows a different, visibly piecewise thrust history. Both CSVs use the same ground-station accelerometer conversion convention. This suggests checking the actual motor thrust curve, flight mass, drag and atmospheric setup before attributing the altitude deficit to the airbrake translation. The files do not uniquely identify which physical parameter is wrong.

**2. Opening time agrees; the commanded opening does not.** The telemetry first shows opening at 14.04 s in GS1 and 14.05 s in OR, indistinguishable at this sampling resolution. GS1 reaches **525 µs**, the declared fully open endpoint. OR reaches only **670 µs**, approximately **65.1%** of the closed-to-open pulse span `(941−670)/(941−525)`. That percentage is a command/linkage proxy, not a measurement of real exposed area.

GS1 then retracts from full opening, while the simulated command varies more gently. The differing altitude and velocity inputs can themselves produce different controller outputs. Moreover, the real trace reaches a higher barometric altitude despite commanding more initial braking, reinforcing that the discrepancy cannot be explained solely by insufficient simulated brake opening.

The GS1 barometer drops about **159 m between 14.040 and 14.388 s**, coinciding with deployment, before resuming its rise. That local transient is preserved in the plot. It is consistent with a pressure-port response, but the available files do not prove the cause. The current simulator supplies ideal atmospheric pressure and does not model local port-pressure disturbances. This local caveat does not change the choice to prioritize the barometer for the overall trajectory.

**3. Descent/recovery differs substantially.** At 100 s, the one-second barometric slope is about **−31.2 m/s** for GS1 and **−42.7 m/s** for OR. The simulated rocket reaches the ground at **131.942 s**, with a logged vertical velocity of **−39.824 m/s**. At that same time GS1's barometer still indicates approximately **1125 m above its prelaunch baseline**. GS1 recording ends at **159.174 s**, still **176 m** above that baseline; a recorded landing time cannot be determined from this file.

The simulation log explicitly labels pyros as recorded only, with recovery controlled by OpenRocket. Recovery configuration and descent aerodynamics therefore need comparison with the real flight. The earlier `MAIN` event is also consistent with the simulated descent reaching its height gate sooner. At the first GS1 `MAIN` packet, reported barometer altitude is 650 m and GPS altitude 455 m; the latter satisfies the selected firmware's alternative GPS height gate. At the simulated transition, the barometer satisfies the gate. Thus the different sensor channels can affect event timing even though GPS is not used as our altitude reference.

**4. The horizontal path is very different.** Both 3D paths are expressed as east/north displacement from their own prelaunch GPS origin. The simulation starts at **28.61°, −80.6°**; GS1 starts near **42.7042295°, −77.1919149°**. Overlaying their absolute coordinates would conflate the different launch sites with trajectory error.

At their respective last samples, GS1 is **3.054 km from launch** (2.517 km east, 1.729 km south), while OR is **68.8 m from launch**. These endpoints occur at different times and GS1 is still airborne. At the common time of OR's last telemetry sample, GS1 is already about **2.854 km from launch**. The simulation's northward displacement is nearly zero. Launch-site settings, wind direction/profile, initial attitude and descent dynamics need attention before calling this a matched flight simulation. Early GPS horizontal fixes also have gaps and a visible position jump; those are shown without smoothing them away.

**5. FC estimates and raw sensor realism need separate treatment.** Around 100 s, GS1's integrated velocity is approximately **−276 m/s**, whereas its barometer indicates a descent near **31 m/s**. The simulated FC estimate is also biased relative to barometric slope, approximately −69 versus −43 m/s. Integrated body-axis acceleration with fixed gravity subtraction should therefore not be treated as true vertical velocity during descent.

After removing the three inconsistent packets, GS1 has **439 samples** reaching the X-gyro's approximately ±1000°/s limit during flight, with additional Y/Z saturation. OR has none. The supplied simulation log states that physical roll coupling and sensor noise are off. The recorded rates and saturation warrant checking attitude/mounting and sensor behavior separately from the altitude comparison.

The simulated CSV's negative temperatures also have a known export explanation: it synthesizes raw counts with the FC calibration constants and decodes them with the historical ground-station constants. The log reports **15°C** at the pad while the CSV shows **−16.15°C**. GS1 reports about **30.93°C** before launch. The diagnostic figure shows the simulation's engineering temperature separately. This decoder mismatch does not numerically convert the already-transmitted `barofilteredalt` field, and cannot simply be subtracted from altitude to fix the trajectory.

## Data quality and reproducibility

Inputs are [the supplied telemetry](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01/build/libs/fc-telemetry/zephyrus-2679935281387596436/telemetry.csv), [the supplied OR log](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/clone/openrocket-release-24.12.RC.01/build/libs/fc-telemetry/zephyrus-2679935281387596436/OR.log), and [GS1](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/benchmarking/telem/ZEPH_TEST_FLIGHT_GS1.csv). They were read without modification. No new simulation or model tuning was performed.

The supplied simulation contains **2215 packets**, all of which pass the existing independent binary/CSV decoder and checksum validation. GS1 contains **4366 rows**. Three GS1 rows are inconsistent with the surrounding flight and are excluded as whole packets from plotted metrics:

| GS1 CSV line, including header | Aligned time | Evidence |
| --- | ---: | --- |
| 2595 | 46.446 s | GPS altitude approximately `7.05e22 m` |
| 3349 | 92.250 s | Isolated latitude jump to 43.112°, plus an isolated 429 µs airbrake pulse |
| 3947 | 130.044 s | Latitude 13.743°, integrated velocity −71487 m/s, packet counter jumping backwards and an isolated barometer dip |

The complete values are retained in [excluded_packets.csv](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/benchmarking/comparison-20260920-2679935281387596436/excluded_packets.csv). Other barometer transients are retained. GPS fixes other than 3 are excluded from horizontal tracks. GPS altitude remains unsettled through the recovery of a stable 3D fix at 14.154 s and is excluded from the low-confidence altitude diagnostic before then. It is never used to choose, rescale or correct the primary height trace.

Plot lines break across invalid observations and telemetry gaps longer than 0.5 s. Interpolated comparison metrics use the same gap limit and never extrapolate. Barometric velocity is a local one-second linear fit with at least eight observations; it is a derived, smoothed quantity. Full method details, source hashes, event log line numbers and numerical results are in [metrics.json](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/benchmarking/comparison-20260920-2679935281387596436/metrics.json).

## Run and rotate the Matplotlib 3D plot

The reusable code is [compare_flight.py](/Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/benchmarking/comparison-20260920-2679935281387596436/compare_flight.py). It requires NumPy and Matplotlib, reads the input CSV/log files, generates all figures and metrics, and optionally opens the 3D figure with mouse rotation, panning and zooming.

```sh
cd /Users/mdn/Developer/ActiveControl_MIT_RktTeam/sim/FCsim/benchmarking/comparison-20260920-2679935281387596436
python3 -m pip install -r requirements.txt
python3 compare_flight.py --show-3d
```

Use your preferred Python environment for installation. To export figures without opening a window, omit `--show-3d`. A GUI-enabled Matplotlib backend is needed for mouse interaction. Static PNG/SVG generation was executed successfully and the macOS interactive backend imported successfully; the interactive window was not opened during this analysis.

For another run, pass `--sim /path/telemetry.csv --log /path/OR.log --gs /path/GS1.csv --output /path/results`. The time origin is inferred from the FC log and first GS1 flight packet. The explicit plausibility bounds and vertical-GPS qualification rules in this script were selected for these supplied files; review them before applying it to a different flight.

For the next modeling iteration, first reconcile the launch setup and boost profile, then recovery/wind behavior, then evaluate controller output with those inputs matched. Preserve the raw barometric reference and its pad offset throughout that work.
