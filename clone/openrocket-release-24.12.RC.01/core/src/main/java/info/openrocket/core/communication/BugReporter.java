package info.openrocket.core.communication;

import java.io.IOException;
import java.io.OutputStreamWriter;
import java.net.HttpURLConnection;
import java.net.URI;
import java.nio.charset.StandardCharsets;

import info.openrocket.core.util.BuildProperties;

public class BugReporter extends Communicator {

	// Inhibit instantiation
	private BugReporter() {
	}

	/** A short draft link; the complete report is copied or saved separately to avoid URI size limits. */
	public static URI getEmailReportURI() {
		String email = BuildProperties.getBugReportEmail();
		if (email.isBlank()) {
			throw new IllegalStateException("No bug report email address is configured");
		}
		String edition = BuildProperties.isMitEdition() ? "OpenRocket MIT " + BuildProperties.getMitVersion()
				: "OpenRocket " + BuildProperties.getVersion();
		return URI.create("mailto:" + email + "?subject=" + encode(edition + " bug report").replace("+", "%20"));
	}

	/**
	 * Send the provided report through the legacy HTTP service in upstream builds.
	 * MIT builds reject this operation before opening a connection; their reports
	 * are shared by email instead.
	 * 
	 * @param report the report to send.
	 * @throws IOException if HTTP reporting is disabled, the connection fails, or
	 *                     the server responds with a wrong response code.
	 */
	public static void sendBugReport(String report) throws IOException {
		if (BuildProperties.isMitEdition()) {
			throw new IOException("Automatic HTTP bug reporting is disabled for OpenRocket MIT. Email the report to "
					+ BuildProperties.getBugReportEmail() + ".");
		}
		sendBugReport(report, BUG_REPORT_URL);
	}

	// Legacy transport, kept separate so its protocol can be tested without enabling it in MIT builds.
	static void sendBugReport(String report, String url) throws IOException {
		HttpURLConnection connection = connectionSource.getConnection(url);

		connection.setConnectTimeout(CONNECTION_TIMEOUT);
		connection.setInstanceFollowRedirects(true);
		connection.setRequestMethod("POST");
		connection.setUseCaches(false);
		connection.setRequestProperty("X-OpenRocket-Version", encode(BuildProperties.getVersion()));

		String post;
		post = (VERSION_PARAM + "=" + encode(BuildProperties.getVersion())
				+ "&" + BUG_REPORT_PARAM + "=" + encode(report));

		OutputStreamWriter wr = null;
		try {
			// Send post information
			connection.setDoOutput(true);
			wr = new OutputStreamWriter(connection.getOutputStream(), StandardCharsets.UTF_8);
			wr.write(post);
			wr.flush();

			if (connection.getResponseCode() != BUG_REPORT_RESPONSE_CODE) {
				throw new IOException("Server responded with code " +
						connection.getResponseCode() + ", expecting " + BUG_REPORT_RESPONSE_CODE);
			}
		} finally {
			if (wr != null)
				wr.close();
			connection.disconnect();
		}
	}

}
