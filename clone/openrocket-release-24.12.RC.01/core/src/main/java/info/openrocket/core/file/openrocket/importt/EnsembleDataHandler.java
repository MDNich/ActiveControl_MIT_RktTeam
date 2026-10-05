package info.openrocket.core.file.openrocket.importt;

import info.openrocket.core.file.DocumentLoadingContext;
import info.openrocket.core.file.simplesax.*;
import info.openrocket.core.logging.WarningSet;
import info.openrocket.core.simulation.*;
import info.openrocket.core.simulation.ensemble.*;
import java.util.*;

final class EnsembleDataHandler extends AbstractElementHandler {
    private final SingleSimulationHandler simulation;
    private final DocumentLoadingContext context;
    private final Map<String, String> attributes;
    private final List<FlightDataBranch> deviations = new ArrayList<>();
    private final Map<Integer, Map<EnsembleMetric, double[]>> samples = new HashMap<>();
    private FlightDataBranchHandler branch;
    private boolean invalid;
    private EnsembleRunArchive.Builder runArchive;
    private EnsembleRunLoader.Handler runHandler;
    private boolean invalidRuns;
    EnsembleDataHandler(SingleSimulationHandler simulation, DocumentLoadingContext context, Map<String, String> attributes) {
        this.simulation=simulation;this.context=context;this.attributes=Map.copyOf(attributes);
    }
    @Override public ElementHandler openElement(String element, HashMap<String,String> a, WarningSet warnings) throws org.xml.sax.SAXException {
        if (element.equals("ensemblerun")) {
            try {
                if (runArchive == null) runArchive = new EnsembleRunArchive.Builder();
                return runHandler = new EnsembleRunLoader.Handler(simulation, context, a);
            } catch (java.io.IOException e) { throw new org.xml.sax.SAXException("Cannot retain archived ensemble runs", e); }
        }
        if (element.equals("samples")) return PlainTextHandler.INSTANCE;
        if (element.equals("databranch") && a.get("name")!=null && a.get("types")!=null) {
            branch=new FlightDataBranchHandler(a.get("name"),a.get("types"),simulation,context);return branch;
        }
        warnings.add("Unknown ensemble element: "+element);return null;
    }
    @Override public void closeElement(String element, HashMap<String,String> a, String content, WarningSet warnings) {
        if (element.equals("ensemblerun")) {
            try { runArchive.add(runHandler.parameters, runHandler.flight()); }
            catch (java.io.IOException | RuntimeException e) { invalidRuns = true; }
            finally { runHandler = null; }
            return;
        }
        try {
            if (element.equals("databranch")) deviations.add(branch.getBranch());
            else if (element.equals("samples")) {
                int b=Integer.parseInt(a.get("branch"));
                var metric=EnsembleMetric.valueOf(a.get("metric"));
                double[] values=content.isBlank()?new double[0]:Arrays.stream(content.trim().split(",")).mapToDouble(Double::parseDouble).toArray();
                if (samples.computeIfAbsent(b,k->new EnumMap<>(EnsembleMetric.class)).put(metric,values)!=null) invalid=true;
            }
        } catch (RuntimeException e) { invalid=true; }
    }
    void apply(FlightData data, WarningSet warnings) {
        try {
            if (invalid || deviations.size()!=data.getBranchCount()) throw new IllegalArgumentException("Invalid ensemble branches or samples");
            var settings=EnsembleSettings.fromAttributes(attributes);
            var values=new ArrayList<Map<EnsembleMetric,double[]>>();
            for (int b=0;b<deviations.size();b++) {
                if (!Objects.equals(deviations.get(b).get(FlightDataType.TYPE_TIME),data.getBranch(b).get(FlightDataType.TYPE_TIME)))
                    throw new IllegalArgumentException("Ensemble time grid differs from mean trajectory");
                var row=samples.get(b);
                if (row==null || row.size()!=EnsembleMetric.values().length) throw new IllegalArgumentException("Incomplete ensemble summaries");
                values.add(row);
            }
            EnsembleRunArchive archive = null;
            if (runArchive != null) {
                if (invalidRuns || runArchive.size() != settings.runs()) {
                    warnings.add("Individual ensemble runs are incomplete; only the aggregate results were loaded.");
                } else {
                    try {
                        archive = runArchive.finish();
                        System.out.println("ENSEMBLE loaded full_results=" + archive.size());
                    }
                    catch (java.io.IOException e) { warnings.add("Individual ensemble runs could not be retained: " + e.getMessage()); }
                }
            }
            data.setEnsembleResult(new EnsembleResult(settings,deviations,values,archive));
        } catch (RuntimeException e) { warnings.add("Ensemble statistics could not be loaded: "+e.getMessage()); }
        finally {
            if (runArchive != null) {
                try { runArchive.close(); }
                catch (java.io.IOException e) { warnings.add("Could not close ensemble run storage: " + e.getMessage()); }
            }
        }
    }
}
