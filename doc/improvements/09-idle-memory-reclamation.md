# Idle memory investigation and fix

11 October 2026. Trigger: OpenRocket showed **11.18 GB** in Activity Monitor after several simulations and then sitting idle.

## Evidence

The original process had exited before it could be inspected. A subsequently opened instance used the bundled Java 17.0.16 with G1, a roughly 30 GiB maximum heap, and periodic garbage collection disabled (`G1PeriodicGCInterval=0`). A live-object histogram after collection contained **178,886,528 bytes**, mostly retained flight samples (`Double` values). The process then had roughly 175 MiB of used Java heap. This does not establish what every byte in the original screenshot represented, but it shows the large process footprint was not all live simulation objects in the inspected instance.

The most recently opened file in OpenRocket's preferences was `20267.demonstrator.iteration3.ork`. It is a 96.8 MB compressed file containing 293.6 MB of XML, saved ensemble runs, and three simulations. Its FC result has about 46,700 points and 98 channels. The reproduction used that file without changing or saving it, and reran **FC Sim** three times using its existing Balius definition.

With explicit collections for diagnosis, live heap after the runs was **168.5, 168.5, and 168.3 MB**, and weak references confirmed that every replaced flight result was collected. A separate native Swing test repeated simulations and opened/closed editors four times; all retired dialogs and flight results were collectible, while the current result remained available.

With the normal heap settings and no explicit collections, the reproduction retained **3.12 GB of committed heap** throughout 45 seconds of idle time. Loading and simulating caused substantial temporary allocation, and G1 did not proactively reclaim it while the application sat idle.

With the new startup policy, a comparable run went from **3.27 GB committed / 612 MB used** after simulation to **1.29 GB committed / 352 MB used** during idle, without an explicit collection. The periodic concurrent collection appeared in the GC log about 37 seconds after the last simulation collection; all replaced-result weak references were cleared. These are Java heap measurements, not a claim that the original Activity Monitor reading was reproduced byte-for-byte.

## Change

`StartupMemoryPolicy.configure()` runs at application startup and enables **periodic concurrent G1 collection with a 30-second interval**, allowing unused heap pages to be returned to the operating system without waiting for another large allocation. A collection can occur roughly 30–60 seconds after the last ordinary collection because G1 checks periodically.

- Preserves explicitly configured JVM settings, including an explicit interval of zero.
- Leaves maximum heap size and all saved flight results unchanged.
- Does not introduce a timer that calls stop-the-world `System.gc()` from Swing.
- Applies to the existing macOS/Windows/JAR startup path; other garbage collectors retain their own policies.
- Prints a `MEMORY` startup line stating the applied setting.

A live VM setting was also applied to the reopened instance during diagnosis; subsequent launches use the source change. Restart OpenRocket to load the rebuilt application code.

The JVM mechanism is documented in [OpenJDK JEP 346](https://openjdk.org/jeps/346).

## Verification artifacts

The [measurement program](../../sim/FCsim/MemoryRetentionVerification.java) records heap use, committed capacity, sample counts, and weak-reference retention. Results live in [memory-20261011](../../sim/FCsim/benchmarking/memory-20261011/). The baseline and fixed idle comparisons do not call `System.gc()`; the separately labeled diagnostic run does.

Five automated checks cover the startup policy, respecting explicit tuning, releasing editor subscriptions, repeated simulation/editor lifecycles, and collection of replaced results. The root shadow JAR was rebuilt as `OpenRocket-MIT-v6.4.jar`; the installed MIT app's existing JAR symlink points to the matching compatibility artifact.

A packaged-JAR probe on Windows 11 with Java 17 also confirmed that startup enables the 30-second interval; its output is saved in `windows-policy.log`.

This change addresses the reproduced **idle heap retention**. Large files and concurrent simulations can still temporarily need substantial memory while working. It does not prove that every possible native renderer, extension, or custom Java board is leak-free.

## Installed-app startup correction

The initial rebuilt app failed on launch because the newly added motor-batch action dereferenced a null simulation selection during `SimulationPanel` construction. The earlier dialog tests and memory probe bypassed that main-window path, so they missed the regression. Both the action's enabled-state update and its invocation now handle an empty selection and reject imported flight data.

Two new `SimulationPanelTest` cases reproduced the startup exception before the correction and passed afterward, covering new and populated documents, selection clearing, multiple selections, and external flight data. The batch-dialog test also passed. After rebuilding the root shadow JAR, the actual `/Applications/OpenRocket_MIT.app/Contents/MacOS/JavaApplicationStub` launcher opened the main window and restored `balius_testing.ork` successfully, verified through the native UI. The memory policy's startup message was also present.
