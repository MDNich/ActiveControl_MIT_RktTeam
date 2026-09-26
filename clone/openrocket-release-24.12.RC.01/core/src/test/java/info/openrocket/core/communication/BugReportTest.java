package info.openrocket.core.communication;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.IOException;
import java.net.URI;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;

import info.openrocket.core.util.BuildProperties;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assumptions;

public class BugReportTest {
	private static final String TEST_REPORT_URL = "https://reports.example.invalid/bugs";

	@AfterEach
	void resetConnections() {
		Communicator.setConnectionSource(new DefaultConnectionSource());
	}

	private HttpURLConnectionMock setup() {
		HttpURLConnectionMock connection = new HttpURLConnectionMock();
		Communicator.setConnectionSource(new ConnectionSourceStub(connection));

		connection.setUseCaches(true);
		return connection;
	}

	private void check(HttpURLConnectionMock connection) {
		assertEquals(TEST_REPORT_URL, connection.getTrueUrl());
		assertTrue(connection.getConnectTimeout() > 0);
		assertEquals(BuildProperties.getVersion(), connection.getRequestProperty("X-OpenRocket-Version"));
		assertTrue(connection.getInstanceFollowRedirects());
		assertEquals(connection.getRequestMethod(), "POST");
		assertFalse(connection.getUseCaches());
	}

	@Test
	public void testBugReportSuccess() throws IOException {
		HttpURLConnectionMock connection = setup();
		connection.setResponseCode(Communicator.BUG_REPORT_RESPONSE_CODE);

		String message = "MyMessage\n" +
				"is important\n" +
				"h\u00e4h?";

		BugReporter.sendBugReport(message, TEST_REPORT_URL);

		check(connection);

		String msg = connection.getOutputStreamString();
		assertTrue(msg.contains("version=" + BuildProperties.getVersion()));
		assertTrue(msg.contains(Communicator.encode(message)));
	}

	@Test
	public void testBugReportFailure() throws IOException {
		HttpURLConnectionMock connection = setup();
		connection.setResponseCode(200);

		String message = "MyMessage\n" +
				"is important\n" +
				"h\u00e4h?";

		try {
			BugReporter.sendBugReport(message, TEST_REPORT_URL);
			fail("Exception did not occur");
		} catch (IOException e) {
			// Success
		}

		check(connection);
	}

	@Test
	void mitEditionCannotSendThroughTheLegacyHttpReporter() {
		Assumptions.assumeTrue(BuildProperties.isMitEdition());
		HttpURLConnectionMock connection = setup();
		IOException failure = assertThrows(IOException.class, () -> BugReporter.sendBugReport("private diagnostic report"));
		assertTrue(failure.getMessage().contains(BuildProperties.getBugReportEmail()));
		assertNull(connection.getTrueUrl(), "No network connection may be requested for MIT error reports");
	}

	@Test
	void mitEmailDraftTargetsTheMaintainerAndIdentifiesThePackagedVersion() {
		Assumptions.assumeTrue(BuildProperties.isMitEdition());
		URI draft = BugReporter.getEmailReportURI();
		assertEquals("mailto", draft.getScheme());
		String decoded = URLDecoder.decode(draft.getRawSchemeSpecificPart(), StandardCharsets.UTF_8);
		assertEquals("therobomentors@gmail.com", BuildProperties.getBugReportEmail());
		assertEquals(BuildProperties.getBugReportEmail() + "?subject=OpenRocket MIT "
				+ BuildProperties.getMitVersion() + " bug report", decoded);
		assertFalse(decoded.contains("sourceforge.net"));
		assertFalse(decoded.contains("github.com/openrocket"));
		assertTrue(draft.toASCIIString().length() < 1024);
	}

}
