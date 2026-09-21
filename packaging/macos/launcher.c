/* Universal macOS entry point. Each architecture uses its own bundled Java 17 runtime. */
#include <mach-o/dyld.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int main(int argc, char **argv) {
    char executable[PATH_MAX], resolved[PATH_MAX];
    uint32_t size = sizeof(executable);
    if (_NSGetExecutablePath(executable, &size) != 0 || realpath(executable, resolved) == NULL) {
        perror("Cannot locate OpenRocket application");
        return 1;
    }
    /* Strip the executable name and MacOS directory, leaving Contents. */
    for (int i = 0; i < 2; i++) {
        char *separator = strrchr(resolved, '/');
        if (!separator) return 1;
        *separator = '\0';
    }
#if defined(__arm64__)
    const char *architecture = "arm64";
#elif defined(__x86_64__)
    const char *architecture = "x86_64";
#else
#error Unsupported macOS architecture
#endif
    char java[PATH_MAX], jar[PATH_MAX], icon[PATH_MAX + 32];
    snprintf(java, sizeof(java), "%s/runtime-%s/bin/java", resolved, architecture);
    snprintf(jar, sizeof(jar), "%s/app/OpenRocket.jar", resolved);
    snprintf(icon, sizeof(icon), "-Xdock:icon=%s/Resources/app.icns", resolved);
    char **arguments = calloc((size_t)argc + 20, sizeof(char *));
    if (!arguments) { perror("Cannot allocate launcher arguments"); return 1; }
    int n = 0;
    arguments[n++] = java;
    arguments[n++] = "-Xdock:name=OpenRocket MIT";
    arguments[n++] = icon;
    arguments[n++] = "-Dapple.awt.application.appearance=system";
    arguments[n++] = "-Dfile.encoding=UTF-8";
    arguments[n++] = "--add-exports=java.base/java.lang=ALL-UNNAMED";
    arguments[n++] = "--add-exports=java.desktop/sun.awt=ALL-UNNAMED";
    arguments[n++] = "--add-exports=java.desktop/sun.java2d=ALL-UNNAMED";
    arguments[n++] = "-cp";
    arguments[n++] = jar;
    arguments[n++] = "info.openrocket.swing.startup.OpenRocket";
    for (int i = 1; i < argc; i++) {
        /* Older Launch Services versions can append a process serial number. */
        if (strncmp(argv[i], "-psn_", 5) != 0) arguments[n++] = argv[i];
    }
    execv(java, arguments);
    perror("Cannot start bundled OpenRocket runtime");
    free(arguments);
    return 1;
}
