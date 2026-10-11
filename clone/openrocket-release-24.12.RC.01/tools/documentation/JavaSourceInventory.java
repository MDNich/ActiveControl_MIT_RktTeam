import com.sun.source.tree.*;
import com.sun.source.util.*;
import java.nio.file.*;
import java.util.*;
import javax.tools.*;

/**
 * Uses the JDK parser to inventory declarations without compiling or executing application code.
 */
public final class JavaSourceInventory {
    private static boolean checkOnly;

    /** Emits inventory data unless only documentation coverage was requested. */
    private static void emit(String line) {
        if (!checkOnly) System.out.println(line);
    }

    /** Encodes a string for the JSON-lines inventory. */
    private static String quote(String s) {
        var out = new StringBuilder("\"");
        for (char c : s.toCharArray()) {
            switch (c) {
                case '\\' -> out.append("\\\\");
                case '"' -> out.append("\\\"");
                case '\n' -> out.append("\\n");
                case '\r' -> out.append("\\r");
                case '\t' -> out.append("\\t");
                default -> {
                    if (c < 32) out.append(String.format("\\u%04x", (int) c));
                    else out.append(c);
                }
            }
        }
        return out.append('"').toString();
    }

    /** Encodes a collection of declaration strings as a JSON array. */
    private static String strings(Collection<String> values) {
        return "["
                + String.join(",", values.stream().map(JavaSourceInventory::quote).toList())
                + "]";
    }

    /**
     * Writes class/method metadata and parser-normalized syntax, including attached Javadoc.
     */
    public static void main(String[] args) throws Exception {
        checkOnly = args.length > 1 && args[1].equals("--check");
        var missing = new ArrayList<String>();
        int[] methods = {0};
        List<String> files =
                Files.readAllLines(Path.of(args[0])).stream()
                        .map(String::trim)
                        .filter(s -> !s.isEmpty() && !s.startsWith("#"))
                        .toList();
        JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
        var diagnostics = new DiagnosticCollector<JavaFileObject>();
        try (var manager =
                compiler.getStandardFileManager(
                        diagnostics, null, java.nio.charset.StandardCharsets.UTF_8)) {
            var task =
                    (JavacTask)
                            compiler.getTask(
                                    null,
                                    manager,
                                    diagnostics,
                                    List.of("-proc:none", "--release", "17"),
                                    null,
                                    manager.getJavaFileObjectsFromStrings(files));
            DocTrees docs = DocTrees.instance(task);
            for (var unit : task.parse()) {
                var positions = docs.getSourcePositions();
                String path = Path.of(unit.getSourceFile().toUri()).toString();
                emit(
                        "{\"kind\":\"unit\",\"path\":"
                                + quote(path)
                                + ",\"syntax\":"
                                + quote(unit.toString())
                                + "}");
                new TreePathScanner<Void, Void>() {
                    private final Deque<String> owners = new ArrayDeque<>();
                    private int anonymous;

                    /**
                     * Emits a type and visits nested declarations with an unambiguous owner path.
                     */
                    @Override
                    public Void visitClass(ClassTree tree, Void unused) {
                        String name = tree.getSimpleName().toString();
                        if (name.isEmpty()) name = "anonymous" + (++anonymous);
                        owners.addLast(name);
                        var doc = docs.getDocCommentTree(getCurrentPath());
                        var fields = new ArrayList<String>();
                        for (var member : tree.getMembers())
                            if (member instanceof VariableTree v)
                                fields.add(v.getName() + ":" + v.getType());
                        emit(
                                "{\"kind\":\"class\",\"path\":"
                                        + quote(path)
                                        + ",\"owner\":"
                                        + quote(String.join(".", owners))
                                        + ",\"package\":"
                                        + quote(
                                                unit.getPackageName() == null
                                                        ? ""
                                                        : unit.getPackageName().toString())
                                        + ",\"declaration\":"
                                        + quote(tree.getKind().toString())
                                        + ",\"extends\":"
                                        + quote(
                                                tree.getExtendsClause() == null
                                                        ? ""
                                                        : tree.getExtendsClause().toString())
                                        + ",\"implements\":"
                                        + strings(
                                                tree.getImplementsClause().stream()
                                                        .map(Object::toString)
                                                        .toList())
                                        + ",\"fields\":"
                                        + strings(fields)
                                        + ",\"start\":"
                                        + positions.getStartPosition(unit, tree)
                                        + ",\"doc\":"
                                        + quote(doc == null ? "" : doc.toString())
                                        + "}");
                        super.visitClass(tree, unused);
                        owners.removeLast();
                        return null;
                    }

                    /** Emits every explicit method, constructor and anonymous-class callback. */
                    @Override
                    public Void visitMethod(MethodTree tree, Void unused) {
                        var doc = docs.getDocCommentTree(getCurrentPath());
                        long start = positions.getStartPosition(unit, tree),
                                end = positions.getEndPosition(unit, tree);
                        String name = tree.getName().toString();
                        methods[0]++;
                        if (doc == null || doc.getFullBody().isEmpty()) {
                            missing.add(
                                    path
                                            + ":"
                                            + unit.getLineMap().getLineNumber(start)
                                            + " "
                                            + String.join(".", owners)
                                            + "."
                                            + name);
                        }
                        emit(
                                "{\"kind\":\"method\",\"path\":"
                                        + quote(path)
                                        + ",\"owner\":"
                                        + quote(String.join(".", owners))
                                        + ",\"name\":"
                                        + quote(name)
                                        + ",\"start\":"
                                        + start
                                        + ",\"end\":"
                                        + end
                                        + ",\"return\":"
                                        + quote(
                                                tree.getReturnType() == null
                                                        ? ""
                                                        : tree.getReturnType().toString())
                                        + ",\"parameters\":"
                                        + strings(
                                                tree.getParameters().stream()
                                                        .map(p -> p.getName() + ":" + p.getType())
                                                        .toList())
                                        + ",\"typeParameters\":"
                                        + strings(
                                                tree.getTypeParameters().stream()
                                                        .map(p -> p.getName().toString())
                                                        .toList())
                                        + ",\"throws\":"
                                        + strings(
                                                tree.getThrows().stream()
                                                        .map(Object::toString)
                                                        .toList())
                                        + ",\"modifiers\":"
                                        + quote(tree.getModifiers().toString())
                                        + ",\"body\":"
                                        + quote(
                                                tree.getBody() == null
                                                        ? ""
                                                        : tree.getBody().toString())
                                        + ",\"doc\":"
                                        + quote(doc == null ? "" : doc.toString())
                                        + "}");
                        return super.visitMethod(tree, unused);
                    }
                }.scan(unit, null);
            }
        }
        if (checkOnly) {
            System.out.println(
                    "Javadoc coverage: "
                            + (methods[0] - missing.size())
                            + "/"
                            + methods[0]
                            + " explicit methods and constructors across "
                            + files.size()
                            + " files");
            if (!missing.isEmpty())
                throw new IllegalStateException(
                        "Missing method Javadoc:\n" + String.join("\n", missing));
        }
        for (var diagnostic : diagnostics.getDiagnostics())
            if (diagnostic.getKind() == Diagnostic.Kind.ERROR)
                throw new IllegalStateException(diagnostic.toString());
    }
}
