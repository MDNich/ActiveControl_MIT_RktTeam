package info.openrocket.core.communication;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import jakarta.json.JsonArray;
import jakarta.json.JsonObject;

/**
 * MIT-edition release metadata from the GitHub releases API.
 */
public class MitReleaseInfo {
	private static final Pattern SHA256_DIGEST_PATTERN = Pattern.compile("^(?:sha256:)?([0-9a-fA-F]{64})$");

	private final JsonObject obj;
	private final List<Asset> assets;

	public MitReleaseInfo(JsonObject obj) {
		this.obj = Objects.requireNonNull(obj, "JsonObject cannot be null");
		this.assets = parseAssets(obj);
	}

	public String getTagName() {
		return getString(obj, "tag_name");
	}

	public String getReleaseName() {
		String name = getString(obj, "name");
		if (name == null || name.isBlank()) {
			return getTagName();
		}
		return name;
	}

	public String getReleaseVersion() {
		return MitUpdateInfoRetriever.normalizeVersion(getTagName());
	}

	public String getReleaseNotes() {
		return getString(obj, "body");
	}

	public String getReleaseURL() {
		return getString(obj, "html_url");
	}

	public List<Asset> getAssets() {
		return assets;
	}

	public Asset getJarAsset() {
		Asset firstJar = null;
		for (Asset asset : assets) {
			String lowerName = asset.name().toLowerCase(Locale.ROOT);
			if (!lowerName.endsWith(".jar")) {
				continue;
			}
			if (firstJar == null) {
				firstJar = asset;
			}
			if (lowerName.contains("mit")) {
				return asset;
			}
		}
		return firstJar;
	}

	public Asset getSha256AssetFor(Asset jarAsset) {
		if (jarAsset == null) {
			return null;
		}

		String jarName = jarAsset.name();
		List<Asset> shaAssets = new ArrayList<>();
		for (Asset asset : assets) {
			String lowerName = asset.name().toLowerCase(Locale.ROOT);
			if (lowerName.endsWith(".sha256") || lowerName.endsWith(".sha256.txt")) {
				shaAssets.add(asset);
			}
			if (asset.name().equals(jarName + ".sha256") || asset.name().equals(jarName + ".sha256.txt")) {
				return asset;
			}
		}

		if (shaAssets.size() == 1) {
			return shaAssets.get(0);
		}

		for (Asset asset : shaAssets) {
			if (asset.name().startsWith(jarName)) {
				return asset;
			}
		}
		return null;
	}

	private static List<Asset> parseAssets(JsonObject obj) {
		List<Asset> parsed = new ArrayList<>();
		JsonArray assetArray = obj.getJsonArray("assets");
		if (assetArray == null) {
			return parsed;
		}
		for (int i = 0; i < assetArray.size(); i++) {
			JsonObject assetObj = assetArray.getJsonObject(i);
			String name = getString(assetObj, "name");
			String browserDownloadUrl = getString(assetObj, "browser_download_url");
			String digest = getString(assetObj, "digest");
			long size = assetObj.containsKey("size") ? assetObj.getJsonNumber("size").longValue() : -1;
			if (!name.isBlank() && !browserDownloadUrl.isBlank()) {
				parsed.add(new Asset(name, browserDownloadUrl, digest, size));
			}
		}
		return List.copyOf(parsed);
	}

	private static String getString(JsonObject obj, String key) {
		if (!obj.containsKey(key) || obj.isNull(key)) {
			return "";
		}
		return obj.getString(key, "");
	}

	public record Asset(String name, String browserDownloadUrl, String digest, long size) {
		public String getSha256Digest() {
			if (digest == null || digest.isBlank()) {
				return "";
			}
			Matcher matcher = SHA256_DIGEST_PATTERN.matcher(digest);
			if (!matcher.matches()) {
				return "";
			}
			return matcher.group(1).toLowerCase(Locale.ROOT);
		}
	}
}
