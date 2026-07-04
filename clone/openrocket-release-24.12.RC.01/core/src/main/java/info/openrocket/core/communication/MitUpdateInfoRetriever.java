package info.openrocket.core.communication;

import java.io.StringReader;
import java.util.HashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import info.openrocket.core.util.BuildProperties;

import jakarta.json.Json;
import jakarta.json.JsonObject;
import jakarta.json.JsonReader;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Fetches MIT-edition update information from the MIT fork's GitHub releases.
 */
public class MitUpdateInfoRetriever {
	private Fetcher fetcher = null;
	private volatile boolean cancelled = false;

	private static final Pattern VERSION_PATTERN = Pattern.compile("(\\d+(?:[.-]\\d+)*)");

	public void startFetchUpdateInfo() {
		this.cancelled = false;
		this.fetcher = new Fetcher();
		this.fetcher.setName("MitUpdateInfoFetcher");
		this.fetcher.setDaemon(true);
		this.fetcher.start();
	}

	public void cancel() {
		this.cancelled = true;
		if (this.fetcher != null) {
			this.fetcher.interrupt();
		}
	}

	public boolean isRunning() {
		if (this.fetcher == null) {
			throw new IllegalStateException("Fetcher has not been started");
		}
		return !cancelled && this.fetcher.isAlive();
	}

	public MitUpdateInfo getUpdateInfo() {
		if (this.fetcher == null) {
			throw new IllegalStateException("Fetcher has not been started");
		}
		return this.fetcher.info;
	}

	public static String normalizeVersion(String tagOrVersion) {
		if (tagOrVersion == null) {
			return "";
		}
		Matcher matcher = VERSION_PATTERN.matcher(tagOrVersion);
		if (!matcher.find()) {
			return "";
		}
		return matcher.group(1).replace('-', '.');
	}

	public static int compareVersions(String a, String b) throws UpdateException {
		String normalizedA = normalizeVersion(a);
		String normalizedB = normalizeVersion(b);
		if (normalizedA.isBlank() || normalizedB.isBlank()) {
			throw new UpdateException(String.format("Malformed MIT edition version: '%s' / '%s'", a, b));
		}

		String[] aParts = normalizedA.split("\\.");
		String[] bParts = normalizedB.split("\\.");
		int length = Math.max(aParts.length, bParts.length);
		for (int i = 0; i < length; i++) {
			int aPart = i < aParts.length ? parseVersionPart(aParts[i], normalizedA) : 0;
			int bPart = i < bParts.length ? parseVersionPart(bParts[i], normalizedB) : 0;
			if (aPart != bPart) {
				return Integer.compare(aPart, bPart);
			}
		}
		return 0;
	}

	private static int parseVersionPart(String part, String version) throws UpdateException {
		try {
			return Integer.parseInt(part);
		} catch (NumberFormatException e) {
			throw new UpdateException("Malformed MIT edition version: " + version, e);
		}
	}

	private static class Fetcher extends Thread {
		private static final Logger log = LoggerFactory.getLogger(Fetcher.class);

		private volatile MitUpdateInfo info;

		@Override
		public void run() {
			try {
				this.info = fetchUpdateInfo();
			} catch (UpdateException e) {
				log.info("MIT edition update check failed: {}", e.getMessage());
				this.info = new MitUpdateInfo(e);
			}
		}

		private MitUpdateInfo fetchUpdateInfo() throws UpdateException {
			String currentVersion = BuildProperties.getMitVersion();
			String updateUrl = BuildProperties.getMitUpdateUrl();
			if (currentVersion == null || currentVersion.isBlank()) {
				throw new UpdateException("MIT edition version is not configured");
			}
			if (updateUrl == null || updateUrl.isBlank()) {
				throw new UpdateException("MIT edition update URL is not configured");
			}

			MitReleaseInfo release = retrieveLatestRelease(updateUrl);
			String latestVersion = release.getReleaseVersion();
			if (latestVersion.isBlank()) {
				throw new UpdateException("Latest MIT release tag does not contain a version: " + release.getTagName());
			}

			int comparison = compareVersions(currentVersion, latestVersion);
			if (comparison >= 0) {
				log.info("MIT edition is up to date: current={}, latest={}", currentVersion, latestVersion);
				return new MitUpdateInfo(release, null, null, false);
			}

			MitReleaseInfo.Asset jarAsset = release.getJarAsset();
			if (jarAsset == null) {
				throw new UpdateException("Latest MIT release has no jar asset");
			}
			String expectedSha256 = jarAsset.getSha256Digest();
			MitReleaseInfo.Asset sha256Asset = release.getSha256AssetFor(jarAsset);
			if (expectedSha256.isBlank() && sha256Asset == null) {
				throw new UpdateException("Latest MIT release has no SHA-256 asset for " + jarAsset.name());
			}

			log.info("Found MIT edition update: current={}, latest={}", currentVersion, latestVersion);
			return new MitUpdateInfo(release, jarAsset, sha256Asset, true, expectedSha256);
		}

		private MitReleaseInfo retrieveLatestRelease(String updateUrl) throws UpdateException {
			try {
				Map<String, String> params = new HashMap<>();
				params.put("accept", "application/vnd.github.v3+json");
				String releaseUrl = GitHubAPIUtil.generateUrlWithParameters(updateUrl, params);
				String pageInfo = GitHubAPIUtil.fetchPageInfo(releaseUrl);
				JsonReader reader = Json.createReader(new StringReader(pageInfo));
				JsonObject obj = reader.readObject();
				return new MitReleaseInfo(obj);
			} catch (Exception e) {
				throw new UpdateException("Could not retrieve MIT edition update information", e);
			}
		}
	}

	public static class UpdateException extends Exception {
		public UpdateException(String message) {
			super(message);
		}

		public UpdateException(String message, Throwable cause) {
			super(message, cause);
		}
	}
}
