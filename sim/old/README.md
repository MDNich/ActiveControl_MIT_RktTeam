
## Project Structure:


### `src`
Main source code of the project, following the src-layout pattern. Contains all the Python modules and packages,
as well as a symlink to the java code from OpenRocket.

### `dat`
Data directory containing:
- OpenRocket design files (`.ork`)
- Simulation results (output files and graphs)

### `docs`
Project documentation, built with Sphinx, including:
- API documentation
- Usage guides
- Configuration files

### `out`
External dependencies and compiled assets:
- OpenRocket JAR file
- Other binary dependencies

### Root Files
- `pyproject.toml`: Project configuration and dependencies
- `LICENSE`: Project license
- `README.md`: Project overview and documentation
- `.gitignore`: Git ignore patterns
