package info.openrocket.swing.gui.mission;

import jakarta.json.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.function.*;
import javax.swing.*;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

/** Small typed forms. Text edits are committed together before changing pages or starting work. */
final class MissionForm extends JPanel {
  record Choice(String value, String label) {
    public String toString() {
      return label;
    }
  }

  private final Supplier<JsonObject> source;
  private final BiConsumer<String, JsonValue> write;
  private final Runnable dirty;
  private final List<Runnable> readers = new ArrayList<>();
  private int row;

  MissionForm(Supplier<JsonObject> source, BiConsumer<String, JsonValue> write, Runnable dirty) {
    super(new GridBagLayout());
    this.source = source;
    this.write = write;
    this.dirty = dirty;
    setBorder(BorderFactory.createEmptyBorder(8, 10, 10, 10));
  }

  static JsonValue value(JsonObject source, String path) {
    try {
      return Json.createPointer(path).getValue(source);
    } catch (RuntimeException e) {
      return JsonValue.NULL;
    }
  }

  static String string(JsonValue value) {
    return value instanceof JsonString s
        ? s.getString()
        : value == JsonValue.NULL ? "" : value.toString();
  }

  void commit() {
    for (var reader : readers) reader.run();
  }

  void section(String title) {
    JLabel label = new JLabel(title);
    label.setFont(label.getFont().deriveFont(Font.BOLD));
    GridBagConstraints c = new GridBagConstraints();
    c.gridx = 0;
    c.gridy = row++;
    c.gridwidth = 2;
    c.weightx = 1;
    c.fill = GridBagConstraints.HORIZONTAL;
    c.insets = new Insets(14, 0, 6, 0);
    add(label, c);
  }

  void note(String text) {
    JTextArea label =
        new JTextArea(text) {
          @Override
          public Dimension getPreferredSize() {
            Dimension size = super.getPreferredSize();
            int width = Math.max(80, getWidth() > 0 ? getWidth() : 400);
            var metrics = getFontMetrics(getFont());
            int lines = 1, used = 0;
            for (String word : getText().split(" ")) {
              if (word.contains("\n")) {
                lines++;
                used = 0;
              }
              int length = metrics.stringWidth(word + " ");
              if (used > 0 && used + length > width) {
                lines++;
                used = 0;
              }
              used += length;
            }
            size.height = lines * metrics.getHeight() + 6;
            return size;
          }
        };
    label.addComponentListener(
        new java.awt.event.ComponentAdapter() {
          public void componentResized(java.awt.event.ComponentEvent e) {
            label.revalidate();
          }
        });
    label.setEditable(false);
    label.setOpaque(false);
    label.setLineWrap(true);
    label.setWrapStyleWord(true);
    label.setColumns(1);
    label.setFont(UIManager.getFont("Label.font"));
    GridBagConstraints c = new GridBagConstraints();
    c.gridx = 0;
    c.gridy = row++;
    c.gridwidth = 2;
    c.weightx = 1;
    c.fill = GridBagConstraints.HORIZONTAL;
    c.insets = new Insets(4, 0, 6, 0);
    add(label, c);
  }

  void addRow(String label, JComponent field) {
    GridBagConstraints c = new GridBagConstraints();
    c.gridx = 0;
    c.gridy = row;
    c.anchor = GridBagConstraints.WEST;
    c.insets = new Insets(4, 0, 4, 12);
    add(new JLabel(label), c);
    c.gridx = 1;
    c.weightx = 1;
    c.fill = GridBagConstraints.HORIZONTAL;
    c.insets = new Insets(4, 0, 4, 0);
    add(field, c);
    row++;
  }

  private JTextField input(String path) {
    JTextField field = new JTextField(string(value(source.get(), path)), 18);
    field.setName(path);
    field
        .getDocument()
        .addDocumentListener(
            new DocumentListener() {
              public void insertUpdate(DocumentEvent e) {
                dirty.run();
              }

              public void removeUpdate(DocumentEvent e) {
                dirty.run();
              }

              public void changedUpdate(DocumentEvent e) {
                dirty.run();
              }
            });
    return field;
  }

  JTextField text(String path, String label) {
    JTextField field = input(path);
    readers.add(() -> write.accept(path, Json.createValue(field.getText().trim())));
    addRow(label, field);
    return field;
  }

  JTextField number(String path, String label, double scale, boolean nullable) {
    JTextField field = input(path);
    JsonValue initial = value(source.get(), path);
    if (initial != JsonValue.NULL)
      field.setText(Double.toString(Double.parseDouble(string(initial)) * scale));
    readers.add(
        () -> {
          String text = field.getText().trim();
          if (text.isEmpty() && nullable) {
            write.accept(path, JsonValue.NULL);
            return;
          }
          try {
            double n = Double.parseDouble(text) / scale;
            if (!Double.isFinite(n)) throw new NumberFormatException();
            write.accept(path, Json.createValue(n));
          } catch (NumberFormatException e) {
            field.requestFocusInWindow();
            throw new IllegalArgumentException("Enter a number for " + label);
          }
        });
    addRow(label, field);
    return field;
  }

  JTextField number(String path, String label) {
    return number(path, label, 1, false);
  }

  JCheckBox toggle(String path, String label) {
    JCheckBox field = new JCheckBox(label, value(source.get(), path) == JsonValue.TRUE);
    field.setName(path);
    field.addActionListener(e -> dirty.run());
    readers.add(() -> write.accept(path, field.isSelected() ? JsonValue.TRUE : JsonValue.FALSE));
    addRow("", field);
    return field;
  }

  JComboBox<Choice> choice(String path, String label, String... pairs) {
    var field = new JComboBox<Choice>();
    field.setName(path);
    String selected = string(value(source.get(), path));
    for (int i = 0; i < pairs.length; i += 2) {
      Choice choice = new Choice(pairs[i], pairs[i + 1]);
      field.addItem(choice);
      if (choice.value().equals(selected)) field.setSelectedItem(choice);
    }
    field.addActionListener(e -> dirty.run());
    readers.add(
        () -> {
          if (field.getSelectedItem() != null)
            write.accept(path, Json.createValue(((Choice) field.getSelectedItem()).value()));
        });
    addRow(label, field);
    return field;
  }

  void finish() {
    GridBagConstraints c = new GridBagConstraints();
    c.gridy = row;
    c.weighty = 1;
    c.fill = GridBagConstraints.VERTICAL;
    add(Box.createVerticalGlue(), c);
  }
}
