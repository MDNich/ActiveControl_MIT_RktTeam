package info.openrocket.swing.startup;

import static org.junit.jupiter.api.Assertions.*;

import info.openrocket.core.util.BuildProperties;
import info.openrocket.core.communication.UpdateInfoRetriever;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Assumptions;

class MitUpdateRoutingTest {
    /** Verifies that mit build does not start an upstream update fetcher. */
    @Test
    void mitBuildDoesNotStartAnUpstreamUpdateFetcher() {
        Assumptions.assumeTrue(BuildProperties.isMitEdition());
        assertNull(SwingStartup.startUpdateChecker());
        assertEquals(
                "https://api.github.com/repos/MDNich/ActiveControl_MIT_RktTeam/releases/latest",
                BuildProperties.getMitUpdateUrl());
    }

    /** Verifies that legacy fetcher refuses upstream checks even if called directly. */
    @Test
    void legacyFetcherRefusesUpstreamChecksEvenIfCalledDirectly() {
        Assumptions.assumeTrue(BuildProperties.isMitEdition());
        var failure =
                assertThrows(
                        UpdateInfoRetriever.UpdateInfoFetcher.UpdateCheckerException.class,
                        () -> new UpdateInfoRetriever.UpdateInfoFetcher().runUpdateFetcher());
        assertTrue(failure.getMessage().contains("disabled for the MIT edition"));
    }
}
