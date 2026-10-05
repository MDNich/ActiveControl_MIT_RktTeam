package info.openrocket.core.file.openrocket.importt;

import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.file.DocumentLoadingContext;
import info.openrocket.core.file.simplesax.*;
import info.openrocket.core.logging.WarningSet;
import info.openrocket.core.simulation.FlightData;
import info.openrocket.core.simulation.ensemble.EnsembleRunParameters;
import java.io.*;
import java.util.HashMap;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;

/** Read a full archived flight without adding another simulation entry to the document. */
public final class EnsembleRunLoader {
    private EnsembleRunLoader() { }
    public static FlightData load(InputStream input, OpenRocketDocument document) throws IOException {
        var context = new DocumentLoadingContext();
        context.setOpenRocketDocument(document);
        var simulation = new SingleSimulationHandler(document, context);
        var warnings = new WarningSet();
        Handler[] handler = new Handler[1];
        var root = new AbstractElementHandler() {
            @Override public ElementHandler openElement(String element, HashMap<String,String> attributes, WarningSet w) throws SAXException {
                if (!element.equals("ensemblerun") || handler[0] != null) throw new SAXException("Invalid archived flight");
                return handler[0] = new Handler(simulation, context, attributes);
            }
            @Override public void closeElement(String element, HashMap<String,String> attributes, String content, WarningSet w) { }
        };
        try { SimpleSAX.readXML(new InputSource(input), root, warnings); }
        catch (SAXException | RuntimeException e) { throw new IOException("Could not read archived flight", e); }
        if (handler[0] == null || handler[0].flight() == null || !warnings.isEmpty())
            throw new IOException("Invalid archived flight: " + warnings);
        return handler[0].flight();
    }

    static final class Handler extends AbstractElementHandler {
        final EnsembleRunParameters parameters;
        private final SingleSimulationHandler simulation;
        private final DocumentLoadingContext context;
        private FlightDataHandler data;
        Handler(SingleSimulationHandler simulation, DocumentLoadingContext context, HashMap<String,String> attributes) throws SAXException {
            this.simulation = simulation; this.context = context;
            try { parameters = EnsembleRunParameters.fromAttributes(attributes); }
            catch (RuntimeException e) { throw new SAXException("Invalid archived run parameters", e); }
        }
        @Override public ElementHandler openElement(String element, HashMap<String,String> attributes, WarningSet warnings) throws SAXException {
            if (!element.equals("flightdata") || data != null) throw new SAXException("Invalid archived flight data");
            return data = new FlightDataHandler(simulation, context, false);
        }
        @Override public void closeElement(String element, HashMap<String,String> attributes, String content, WarningSet warnings) { }
        FlightData flight() { return data == null ? null : data.getFlightData(); }
    }
}
