package info.openrocket.core.file.openrocket;

import java.io.File;
import java.io.IOException;
import java.io.InterruptedIOException;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.regex.Pattern;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import info.openrocket.core.appearance.Appearance;
import info.openrocket.core.appearance.DecalImage;
import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.document.StorageOptions;
import info.openrocket.core.logging.ErrorSet;
import info.openrocket.core.logging.WarningSet;
import info.openrocket.core.rocketcomponent.AirbrakeSet;
import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.rocketcomponent.InsideColorComponent;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.rocketcomponent.TabControlledTrapezoidFinSet;
import info.openrocket.core.simulation.customexpression.CustomExpression;

/**
 * Lossy, separate-file export targeting the official OpenRocket 24.12 writer/reader.
 * Streams the existing design without copying flight histories or mutating the document.
 * All simulations are exported as unrun baselines, with no extension/plugin dependencies.
 */
public final class VanillaOpenRocketExporter {
    private static final Pattern MIT_VARIABLE = Pattern.compile(
            "(?<![\\p{L}\\p{N}_])(?:FC_[\\p{L}\\p{N}_]+|Qw|Qx|Qy|Qz|Vx|Vy)(?![\\p{L}\\p{N}_])");

    /** Changes shown before export and logged after successful completion. */
    public static List<String> describeChanges(OpenRocketDocument document) {
        List<String> changes = new ArrayList<>();
        for (RocketComponent component : document.getRocket()) {
            String name = component.getName();
            if (component instanceof TabControlledTrapezoidFinSet)
                changes.add(name + ": roll-control tabs removed; fixed trapezoidal fins retained.");
            if (component instanceof FinSet fins) {
                if (fins.getCrossSection() == FinSet.CrossSection.TRIANGULAR)
                    changes.add(name + ": triangular leading edge replaced by a rounded cross-section."
                            + " Outline, thickness and fillets retained; calculated mass and drag may change.");
                if (Math.abs(fins.getCantAngle()) > VanillaComponentSaver.MAX_CANT)
                    changes.add(name + ": fin cant limited to "
                            + Math.copySign(15, fins.getCantAngle()) + " degrees (vanilla limit).");
            }
            if (component instanceof AirbrakeSet)
                changes.add(name + ": airbrakes replaced by ballast with the same mass and axial CG;"
                        + " deployment, drag and detailed inertia are not retained.");
        }
        if (document.getSimulationCount() > 0) {
            changes.add("All " + document.getSimulationCount() + " simulations exported as unrun baselines."
                    + " Saved results, telemetry and imported flight logs are omitted; rerun in vanilla OpenRocket.");
            int extensions = document.getSimulations().stream()
                    .mapToInt(s -> s.getSimulationExtensions().size()).sum();
            if (extensions > 0)
                changes.add("All " + extensions + " simulation extensions removed, including flight computers,"
                        + " custom Java listeners and scripts. Saved OpenRocket recovery settings are retained;"
                        + " review deployment settings before rerunning.");
            if (document.getSimulations().stream().anyMatch(s -> s.getOptions().getEnsembleSettings().enabled()))
                changes.add("Ensemble settings and individual runs omitted; nominal motor and atmosphere retained.");
            if (document.getSimulations().stream().anyMatch(s -> s.getOptions().getInitialRollRate() != 0))
                changes.add("Initial roll rate reset to zero (unsupported by vanilla).");
        }
        if (!document.getMotorBatches().isEmpty())
            changes.add("Motor-batch history omitted; ordinary motor configurations and simulations retained.");
        Set<CustomExpression> kept = new HashSet<>(compatibleExpressions(document));
        for (CustomExpression expression : document.getCustomExpressions()) {
            if (!kept.contains(expression))
                changes.add("Custom expression '" + expression.getName()
                        + "' omitted because it depends on MIT-only or extension plot variables.");
        }
        return List.copyOf(changes);
    }

    /** Retain vanilla expressions; drop dependent expressions transitively as well. */
    static List<CustomExpression> compatibleExpressions(OpenRocketDocument document) {
        List<CustomExpression> expressions = document.getCustomExpressions();
        Set<String> unavailable = new HashSet<>();
        for (var simulation : document.getSimulations()) {
            for (var extension : simulation.getSimulationExtensions()) {
                for (var type : extension.getFlightDataTypes()) unavailable.add(type.getSymbol());
            }
        }
        Set<CustomExpression> removed = new HashSet<>();
        boolean changed;
        do {
            changed = false;
            for (CustomExpression expression : expressions) {
                if (removed.contains(expression)) continue;
                String formula = expression.getExpressionString();
                if (MIT_VARIABLE.matcher(formula).find() || unavailable.stream().anyMatch(symbol ->
                        !symbol.isEmpty() && Pattern.compile("(?<![\\p{L}\\p{N}_])" + Pattern.quote(symbol)
                                + "(?![\\p{L}\\p{N}_])").matcher(formula).find())) {
                    removed.add(expression);
                    unavailable.add(expression.getSymbol());
                    changed = true;
                }
            }
        } while (changed);
        return expressions.stream().filter(e -> !removed.contains(e)).toList();
    }

    public static boolean isSourceFile(OpenRocketDocument document, File destination) throws IOException {
        File source = document.getFile();
        return source != null && (source.getCanonicalFile().equals(destination.getCanonicalFile())
                || (source.exists() && destination.exists()
                && Files.isSameFile(source.toPath(), destination.toPath())));
    }

    /** Writes a complete ZIP beside its destination, then replaces the destination. */
    public void export(OpenRocketDocument document, File destination) throws IOException {
        if (isSourceFile(document, destination))
            throw new IOException("Choose a separate file for vanilla export; the original MIT design must be kept.");
        Path target = destination.toPath().toAbsolutePath();
        Path temporary = Files.createTempFile(target.getParent(), ".openrocket-vanilla-", ".tmp");
        try {
            checkCancelled();
            try (ZipOutputStream zip = new ZipOutputStream(Files.newOutputStream(temporary))) {
                zip.putNextEntry(new ZipEntry("rocket.ork"));
                new OpenRocketSaver(true).save(zip, document, new StorageOptions(), new WarningSet(), new ErrorSet());
                zip.closeEntry();
                // Copy both outside and inside textures, using the original attachment sources.
                Set<DecalImage> decals = new TreeSet<>();
                for (RocketComponent component : document.getRocket()) {
                    addTexture(decals, component.getAppearance());
                    if (component instanceof InsideColorComponent inside)
                        addTexture(decals, inside.getInsideColorComponentHandler().getInsideAppearance());
                }
                for (DecalImage image : decals) {
                    checkCancelled();
                    zip.putNextEntry(new ZipEntry(image.getName()));
                    try (var bytes = image.getBytes()) {
                        byte[] buffer = new byte[8192];
                        int count;
                        while ((count = bytes.read(buffer)) != -1) {
                            checkCancelled();
                            zip.write(buffer, 0, count);
                        }
                    } catch (info.openrocket.core.util.DecalNotFoundException e) {
                        throw new IOException("Cannot export missing texture: " + image.getName(), e);
                    }
                    zip.closeEntry();
                }
            }
            checkCancelled();
            try {
                Files.move(temporary, target, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
            } catch (AtomicMoveNotSupportedException e) {
                Files.move(temporary, target, StandardCopyOption.REPLACE_EXISTING);
            }
            System.out.println("VANILLA_EXPORT OpenRocket 24.12: " + target);
            for (String change : describeChanges(document)) System.out.println("VANILLA_EXPORT " + change);
        } finally {
            Files.deleteIfExists(temporary);
        }
    }

    private static void addTexture(Set<DecalImage> decals, Appearance appearance) {
        if (appearance != null && appearance.getTexture() != null)
            decals.add(appearance.getTexture().getImage());
    }

    private static void checkCancelled() throws InterruptedIOException {
        if (Thread.currentThread().isInterrupted()) throw new InterruptedIOException("Vanilla export cancelled");
    }
}
