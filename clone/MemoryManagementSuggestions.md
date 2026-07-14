# Memory Management Suggestions

## Summary

The most likely post-simulation memory retention is not the normal document delete path by itself. The stronger evidence is that MIT-added simulation listeners and controller classes keep large simulation objects reachable through `static` fields. Because `SimulationStatus.clone()` is shallow, these fields can retain the same `FlightDataBranch`, `SimulationConditions`, and parent `Simulation` even after the simulation is deleted from the document.

## Highest Priority Fixes

1. Avoid storing `SimulationStatus` and `FlightDataBranch` in static fields.

   `SimulationStatus.clone()` is not a detached snapshot. It keeps references to the same large branch and related simulation state. Replace static status snapshots with small DTOs or primitive fields containing only the values needed by the controller logic.

2. Remove or clear `MidControlStepLauncher.datStorage`.

   `ModifiedEventSimulationEngine` assigns the active `FlightDataBranch` to `MidControlStepLauncher.datStorage`. That field is static and can directly pin the full flight data table. Prefer removing this storage entirely. If it is still needed temporarily, clear it in a guaranteed cleanup path after the simulation completes.

3. Clear MIT controller/listener static state after simulation completion.

   Add explicit reset methods for classes such as:

   - `MidControlStepLauncher`
   - `NewControlStepListener`
   - `FlightControllerSimulatorListener`
   - `AirbrakesControllerListener`
   - `RTFC`

   The reset should null object references such as `SimulationStatus`, `FlightDataBranch`, `Rocket`, `FinSet`, `AirbrakeSet`, and large log lists when they are no longer needed.

## Secondary Improvements

1. Treat simulation deletion cleanup as defensive, not the primary fix.

   `OpenRocketDocument.removeSimulation(...)` currently removes the simulation from the document list and fires a change event. That should be enough if no external references exist. Clearing `simulatedData` on delete may help release memory sooner, but it does not fix static retainers and may interfere with UI, export, plot, or undo expectations if done carelessly.

2. Replace per-step `FlightDataBranch.get(type)` calls in controller listeners.

   `DataBranch.get(type)` clones the entire column. In per-step code this creates large transient allocation pressure. Prefer `getLast(type)` when only the latest value is needed, or add a non-cloning indexed accessor for controlled internal use.

3. Review stage-separation branch copying as a memory amplifier.

   `FlightDataBranch(String, RocketComponent, FlightDataBranch)` copies all prior parent rows into the new branch. This can multiply memory for long staged simulations. It is not necessarily a leak, but it can make large simulations much larger than expected.

4. Consider reducing retained flight data resolution.

   Long simulations store many columns of boxed `Double` values. If full-resolution plotting/export is not needed, consider downsampling saved flight data or storing only selected channels.

5. Longer term, use primitive-backed storage for flight data.

   `ArrayList<Double>` has high overhead compared with primitive `double[]` or a primitive double list. A primitive-backed `FlightDataBranch` would substantially reduce memory usage for large runs.

## Verification Plan

1. Run a large simulation.
2. Delete the simulation from the UI.
3. Force a full GC.
4. Take a heap dump.
5. Inspect dominators and retaining paths.

Expected suspicious paths include:

```text
static field -> SimulationStatus -> FlightDataBranch -> values -> ArrayList<Double>
static field -> FlightDataBranch -> values -> ArrayList<Double>
static field -> SimulationStatus -> SimulationConditions -> Simulation -> simulatedData
```

If live heap drops after full GC but the operating system still reports high process memory, that may be JVM committed heap behavior rather than live-object leakage.
