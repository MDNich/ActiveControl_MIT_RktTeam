Mission Simulation height model

egm96-15.pgm is the unmodified GeographicLib distribution of the NGA/NASA
EGM96 15-minute geoid grid. Source URL and SHA-256 are in provenance.json.
NGA EGM96 data are public domain; see the dataset attribution maintained by
PROJ: https://github.com/OSGeo/PROJ-data/blob/master/us_nga/us_nga_README.txt
GeographicLib's MIT license is also included in ../licenses/.

MissionGeoid uses bilinear interpolation of this grid. Header estimates are
1.152 m maximum / 0.040 m RMS interpolation error relative to EGM96, not an
estimate of surveyed ground accuracy. A geoid grid supplies separation N,
not terrain elevation. Enter station height H explicitly; h = H + N.

Coordinate parsing uses NGA MGRS Java 2.1.3 (MIT, license in ../licenses/).
Its UTM conversion reports seven decimal places. Existing stored positions
are retained exactly when the original MGRS notation is reapplied. Polar
positions use latitude/longitude; this MGRS parser covers UTM zones only.
