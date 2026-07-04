package info.openrocket.core.communication;

/**
 * Result of the MIT-edition update check.
 */
public class MitUpdateInfo {
	private final MitReleaseInfo latestRelease;
	private final MitReleaseInfo.Asset jarAsset;
	private final MitReleaseInfo.Asset sha256Asset;
	private final boolean updateAvailable;
	private final Exception exception;

	public MitUpdateInfo(MitReleaseInfo latestRelease, MitReleaseInfo.Asset jarAsset,
			MitReleaseInfo.Asset sha256Asset, boolean updateAvailable) {
		this.latestRelease = latestRelease;
		this.jarAsset = jarAsset;
		this.sha256Asset = sha256Asset;
		this.updateAvailable = updateAvailable;
		this.exception = null;
	}

	public MitUpdateInfo(Exception exception) {
		this.latestRelease = null;
		this.jarAsset = null;
		this.sha256Asset = null;
		this.updateAvailable = false;
		this.exception = exception;
	}

	public MitReleaseInfo getLatestRelease() {
		return latestRelease;
	}

	public MitReleaseInfo.Asset getJarAsset() {
		return jarAsset;
	}

	public MitReleaseInfo.Asset getSha256Asset() {
		return sha256Asset;
	}

	public boolean isUpdateAvailable() {
		return updateAvailable;
	}

	public Exception getException() {
		return exception;
	}
}
