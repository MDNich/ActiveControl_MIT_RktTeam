package info.openrocket.swing.gui.flightcomputer;

import info.openrocket.core.simulation.flightcomputer.*;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import jakarta.json.*;
import javax.swing.*;
import javax.swing.tree.*;
import javax.swing.event.*;
import java.awt.*;
import java.awt.datatransfer.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import java.util.function.Consumer;

/** Graphical editor of a single external .fc document. No design graph is stored in the rocket. */
public final class FlightComputerDesigner extends JPanel {
    private FlightComputerDesign design;
    private Path file;
    private String diskHash, savedText;
    private final Deque<FlightComputerDesign> undo=new ArrayDeque<>(),redo=new ArrayDeque<>();
    private final Simulation simulation;
    private final Consumer<Path> selectedFile;
    private final Canvas canvas=new Canvas();
    private final JPanel properties=new PropertyPanel();
    private final DefaultTreeModel treeModel=new DefaultTreeModel(new DefaultMutableTreeNode("Hardware"));
    private final JTree tree=new info.openrocket.swing.gui.components.BasicTree();
    private final DefaultListModel<FlightComputerDesign.Diagnostic> diagnostics=new DefaultListModel<>();
    private final JLabel status=new JLabel();
    private final JTabbedPane tabs=new JTabbedPane();
    private final JPanel behavior=new JPanel(new BorderLayout(8,8));
    private String selected="",ruleId="launch";
    private final Set<String> multiSelection=new LinkedHashSet<>();
    private final JList<String> rules=new JList<>();
    private final JPanel stateStrip=new JPanel(new FlowLayout(FlowLayout.LEFT,8,8));
    private String selectedState="apogee";
    private boolean editingState=true;
    private final JPanel conditionHolder=new JPanel(new BorderLayout());
    private final JTextArea testLog=new JTextArea();
    private final TracePlot tracePlot=new TracePlot();
    private JButton undoButton,redoButton;
    private boolean refreshing;
    private Runnable applyProperties=()->{};
    private Runnable pendingForm;
    private void applyPendingForm(){var action=pendingForm;if(action!=null){action.run();pendingForm=null;}}

    public static void open(Component owner,Path path,Simulation simulation,Consumer<Path> selected) {
        try {
            var editor=new FlightComputerDesigner(path,simulation,selected);
            var dialog=new JDialog(SwingUtilities.getWindowAncestor(owner),"Flight computer designer",Dialog.ModalityType.MODELESS);
            dialog.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
            dialog.setContentPane(editor);dialog.setSize(1280,850);dialog.setMinimumSize(new Dimension(920,620));dialog.setLocationRelativeTo(owner);
            dialog.addWindowListener(new WindowAdapter(){@Override public void windowClosing(WindowEvent e){if(editor.confirmClose())dialog.dispose();}});
            dialog.setVisible(true);
        }catch(Exception e){JOptionPane.showMessageDialog(owner,e.getMessage(),"Flight computer",JOptionPane.ERROR_MESSAGE);}
    }
    public FlightComputerDesigner(Path path,Simulation simulation,Consumer<Path> selectedFile) throws IOException {
        super(new BorderLayout(8,8));setBorder(BorderFactory.createEmptyBorder(8,8,8,8));
        this.file=path;this.simulation=simulation;this.selectedFile=selectedFile;
        design=FlightComputerDesign.read(path);savedText=design.text();diskHash=FlightComputerDesign.hash(Files.readAllBytes(path));
        var toolbar=new JToolBar();toolbar.setFloatable(false);
        button(toolbar,"Save",()->{applyPendingForm();save(false);});button(toolbar,"Save as…",()->{applyPendingForm();save(true);});button(toolbar,"Export…",this::export);
        button(toolbar,"Reload",this::reload);toolbar.addSeparator();
        undoButton=button(toolbar,"Undo",()->history(true));redoButton=button(toolbar,"Redo",()->history(false));
        button(toolbar,"Validate",()->{refresh();tabs.setSelectedIndex(0);});button(toolbar,"Fit",()->canvas.fit());
        button(toolbar,"Library folder",()->{try{Desktop.getDesktop().open(FlightComputerLibrary.directory().toFile());}catch(Exception e){error(e);}});
        add(toolbar,BorderLayout.NORTH);
        tree.setModel(treeModel);tree.setName("fc.designer.tree");tree.setRowHeight(0);
        tree.setCellRenderer(new DefaultTreeCellRenderer(){
            public Component getTreeCellRendererComponent(JTree t,Object value,boolean sel,boolean expanded,boolean leaf,int row,boolean focus){
                super.getTreeCellRendererComponent(t,value,sel,expanded,leaf,row,focus);
                if(value instanceof DefaultMutableTreeNode node && node.getUserObject() instanceof Item item){
                    var component=design.node(item.id);setIcon(FlightComputerIcons.of(component==null?"design":component.getString("type")));
                }
                setIconTextGap(4);return this;
            }
        });
        var library=new JPanel(new BorderLayout(4,4));
        var palette=new JList<>(FlightComputerModels.TYPES.toArray(String[]::new));palette.setName("fc.designer.palette");palette.setSelectedIndex(0);
        palette.setCellRenderer(new DefaultListCellRenderer(){public Component getListCellRendererComponent(JList<?> list,Object value,int index,boolean sel,boolean focus){super.getListCellRendererComponent(list,FlightComputerModels.label(value.toString()),index,sel,focus);setIcon(FlightComputerIcons.of(value.toString()));setIconTextGap(4);return this;}});
        palette.setDragEnabled(!GraphicsEnvironment.isHeadless());palette.setTransferHandler(new TransferHandler(){
            @Override public int getSourceActions(JComponent c){return COPY;}
            @Override protected Transferable createTransferable(JComponent c){return new StringSelection(palette.getSelectedValue());}
        });
        var paletteBox=new JPanel(new BorderLayout());paletteBox.add(new JLabel("Component library — drag to canvas"),BorderLayout.NORTH);paletteBox.add(new JScrollPane(palette));
        var add=new JButton("Add selected component");add.addActionListener(e->addNode(palette.getSelectedValue(),80,100));paletteBox.add(add,BorderLayout.SOUTH);
        var left=new JSplitPane(JSplitPane.VERTICAL_SPLIT,paletteBox,new JScrollPane(tree));left.setResizeWeight(.48);library.add(left);
        tree.addTreeSelectionListener(e->{if(!refreshing&&tree.getLastSelectedPathComponent() instanceof DefaultMutableTreeNode n&&n.getUserObject() instanceof Item item)select(item.id);});
        var center=new JPanel(new BorderLayout());
        var actions=new JToolBar();actions.setFloatable(false);button(actions,"Connect…",this::connect);button(actions,"Duplicate",this::duplicate);button(actions,"Delete",this::delete);
        button(actions,"Collapse / expand",this::collapse);button(actions,"Align row",this::align);
        canvas.setToolTipText("Drag to move · Middle/Alt-drag to pan · Mouse wheel to zoom · Shift-click to select several · Click a connection to inspect it");center.add(actions,BorderLayout.NORTH);center.add(canvas);
        properties.setLayout(new BoxLayout(properties,BoxLayout.Y_AXIS));
        var right=new JScrollPane(properties);right.setPreferredSize(new Dimension(280,400));
        var centerRight=new JSplitPane(JSplitPane.HORIZONTAL_SPLIT,center,right);centerRight.setResizeWeight(.75);
        var hardware=new JSplitPane(JSplitPane.HORIZONTAL_SPLIT,library,centerRight);hardware.setDividerLocation(215);hardware.setResizeWeight(0);
        tabs.addTab("Hardware",hardware);buildBehavior();tabs.addTab("Behavior & timing",behavior);tabs.addTab("Test & traces",buildTests());add(tabs);
        var list=new JList<>(diagnostics);list.setVisibleRowCount(3);list.addListSelectionListener(e->{if(!e.getValueIsAdjusting()&&list.getSelectedValue()!=null){select(list.getSelectedValue().item());tabs.setSelectedIndex(0);}});
        var bottom=new JPanel(new BorderLayout(3,3));bottom.add(new JScrollPane(list));bottom.add(status,BorderLayout.SOUTH);add(bottom,BorderLayout.SOUTH);
        bindKey("control Z","undo",()->history(true));bindKey("meta Z","macUndo",()->history(true));bindKey("control Y","redo",()->history(false));bindKey("meta shift Z","macRedo",()->history(false));
        for(String modifier:List.of("control", "meta")){
            bindKey(modifier+" S","save-"+modifier,()->{applyPendingForm();save(false);});
            bindKey(modifier+" shift S","saveAs-"+modifier,()->{applyPendingForm();save(true);});
            bindKey(modifier+" D","duplicate-"+modifier,()->{if(!editingText())duplicate();});
            bindKey(modifier+" shift F","fit-"+modifier,canvas::fit);
            bindKey(modifier+" ENTER","apply-"+modifier,this::applyPendingForm);
        }
        bindKey("F2","rename",this::focusName);
        for(JComponent surface:List.of(tree,canvas)){
            surface.getInputMap().put(KeyStroke.getKeyStroke("DELETE"),"deleteComponent");
            surface.getInputMap().put(KeyStroke.getKeyStroke("BACK_SPACE"),"deleteComponent");
            surface.getActionMap().put("deleteComponent",new AbstractAction(){public void actionPerformed(ActionEvent e){delete();}});
        }
        button(toolbar,"Shortcuts",()->JOptionPane.showMessageDialog(this,
            "Ctrl / ⌘ S: apply properties and save\nCtrl / ⌘ Shift S: save a copy\nCtrl / ⌘ Z: undo · Shift Z / Ctrl Y: redo\nCtrl / ⌘ D: duplicate selected component\nDelete / Backspace: delete selected component (tree/canvas)\nF2: rename · Ctrl / ⌘ Shift F: fit view\nEnter: apply a property · Ctrl / ⌘ Enter: apply all properties\nNotes and Java source: Enter inserts a new line.\nArrow keys navigate the tree; Shift-click selects several canvas items.","Designer shortcuts",JOptionPane.INFORMATION_MESSAGE));
        refresh();
    }
    private boolean editingText(){return KeyboardFocusManager.getCurrentKeyboardFocusManager().getFocusOwner() instanceof javax.swing.text.JTextComponent;}
    private void focusName(){for(var child:properties.getComponents())if(child instanceof Container row)for(var input:row.getComponents())if(input instanceof JTextField text){text.requestFocusInWindow();text.selectAll();return;}}
    private JButton button(JComponent parent,String title,Runnable action){var b=new JButton(title);b.setAlignmentX(Component.LEFT_ALIGNMENT);b.addActionListener(e->{try{action.run();}catch(Exception ex){error(ex);}});parent.add(b);return b;}
    private void bindKey(String stroke,String name,Runnable r){getInputMap(WHEN_ANCESTOR_OF_FOCUSED_COMPONENT).put(KeyStroke.getKeyStroke(stroke),name);getActionMap().put(name,new AbstractAction(){public void actionPerformed(ActionEvent e){r.run();}});}
    private void error(Exception e){JOptionPane.showMessageDialog(this,e.getMessage(),"Flight computer",JOptionPane.ERROR_MESSAGE);}
    public FlightComputerDesign getDesign(){return design;}
    public void edit(FlightComputerDesign value){if(value.json().equals(design.json()))return;undo.push(design);if(undo.size()>100)undo.removeLast();redo.clear();design=value;refresh();}
    private void history(boolean backwards){var from=backwards?undo:redo;var to=backwards?redo:undo;if(!from.isEmpty()){to.push(design);design=from.pop();refresh();}}
    private boolean dirty(){return !savedText.equals(design.text());}
    private boolean confirmClose(){if(!dirty())return true;int answer=JOptionPane.showConfirmDialog(this,"Save this external .fc file before closing?","Unsaved design",JOptionPane.YES_NO_CANCEL_OPTION);return answer==JOptionPane.NO_OPTION||(answer==JOptionPane.YES_OPTION&&save(false));}
    private boolean save(boolean copy){
        try {
            FlightComputerDesign value=design;Path target=file;
            if(copy||FlightComputerLibrary.protectedFile(file)){
                String name=JOptionPane.showInputDialog(this,"Name for the independent design",design.name()+" copy");if(name==null||name.isBlank())return false;
                value=design.copy(name);
                String stem=name.replaceAll("[^A-Za-z0-9._-]","-");target=FlightComputerLibrary.directory().resolve(stem+".fc");
                if(Files.exists(target))target=FlightComputerLibrary.directory().resolve(stem+"-"+value.id().substring(0,8)+".fc");
            } else if(!Files.exists(file)||!diskHash.equals(FlightComputerDesign.hash(Files.readAllBytes(file)))){
                JOptionPane.showMessageDialog(this,"The file changed outside this editor. Reload it or use Save as to preserve both versions.");return false;
            }
            value.write(target);file=target;design=value;savedText=design.text();diskHash=FlightComputerDesign.hash(Files.readAllBytes(file));
            if(selectedFile!=null)selectedFile.accept(file);refresh();System.out.println("FC designer.save path="+file+" semantic_hash="+design.fingerprint());return true;
        }catch(Exception e){error(e);return false;}
    }
    private void export(){var chooser=new JFileChooser();chooser.setSelectedFile(new File(design.name().replaceAll("[^A-Za-z0-9._-]","-")+".fc"));if(chooser.showSaveDialog(this)!=JFileChooser.APPROVE_OPTION)return;
        Path p=chooser.getSelectedFile().toPath();if(!p.toString().toLowerCase(Locale.ROOT).endsWith(".fc"))p=Path.of(p+".fc");
        if(Files.exists(p)&&JOptionPane.showConfirmDialog(this,"Replace "+p+"?","Export",JOptionPane.OK_CANCEL_OPTION)!=JOptionPane.OK_OPTION)return;
        try{design.write(p);}catch(Exception e){error(e);}}
    private void reload(){if(dirty()&&JOptionPane.showConfirmDialog(this,"Discard unsaved edits and reload?","Reload",JOptionPane.OK_CANCEL_OPTION)!=JOptionPane.OK_OPTION)return;
        try{design=FlightComputerDesign.read(file);savedText=design.text();diskHash=FlightComputerDesign.hash(Files.readAllBytes(file));undo.clear();redo.clear();refresh();}catch(Exception e){error(e);}}
    private record Item(String id,String name){public String toString(){return name;}}
    private void refresh(){
        pendingForm=null;refreshing=true;
        try{
            var root=new DefaultMutableTreeNode(new Item("",design.name()));var map=new LinkedHashMap<String,DefaultMutableTreeNode>();
            for(var n:design.nodes().getValuesAs(JsonObject.class))map.put(n.getString("id"),new DefaultMutableTreeNode(new Item(n.getString("id"),n.getString("name"))));
            for(var n:design.nodes().getValuesAs(JsonObject.class)){var parent=map.get(n.getString("board",""));var child=map.get(n.getString("id"));if(parent==null||parent==child)root.add(child);else{try{parent.add(child);}catch(IllegalArgumentException e){root.add(child);}}}
            treeModel.setRoot(root);for(int i=0;i<tree.getRowCount();i++)tree.expandRow(i);
            diagnostics.clear();design.diagnostics().forEach(diagnostics::addElement);
            status.setText((dirty()?"Unsaved · ":"")+file+ (FlightComputerLibrary.protectedFile(file)?" · Protected template; saving creates a copy":" · Saving changes this shared external file"));
            undoButton.setEnabled(!undo.isEmpty());redoButton.setEnabled(!redo.isEmpty());
            showProperties();showCondition();canvas.repaint();
        }finally{refreshing=false;}
    }
    private void select(String id){selected=id;multiSelection.clear();if(!id.isBlank())multiSelection.add(id);showProperties();canvas.repaint();}
    public void addNode(String type,double x,double y){
        String parent="";var current=design.node(selected);if(current!=null)parent=FlightComputerModels.isBoard(current.getString("type"))?selected:current.getString("board","");
        var n=FlightComputerModels.newNode(type,parent,x,y);selected=n.getString("id");edit(design.with("nodes",Json.createArrayBuilder(design.nodes()).add(n).build()));
    }
    private void replaceNode(JsonObject node){var b=Json.createArrayBuilder();for(var n:design.nodes().getValuesAs(JsonObject.class))b.add(n.getString("id").equals(node.getString("id"))?node:n);edit(design.with("nodes",b.build()));}
    private boolean descendant(JsonObject n,String parent){var seen=new HashSet<String>();while(n!=null&&!n.getString("board","").isBlank()){String id=n.getString("board");if(id.equals(parent))return true;if(!seen.add(id))return false;n=design.node(id);}return false;}
    private void delete(){if(selected.isBlank())return;var ids=new HashSet<String>();ids.add(selected);for(var n:design.nodes().getValuesAs(JsonObject.class))if(descendant(n,selected))ids.add(n.getString("id"));
        var nodes=Json.createArrayBuilder();for(var n:design.nodes().getValuesAs(JsonObject.class))if(!ids.contains(n.getString("id")))nodes.add(n);
        var links=Json.createArrayBuilder();for(var c:design.connections().getValuesAs(JsonObject.class))if(!ids.contains(c.getString("id"))&&!ids.contains(c.getString("from"))&&!ids.contains(c.getString("to")))links.add(c);
        edit(new FlightComputerDesign(Json.createObjectBuilder(design.json()).add("nodes",nodes).add("connections",links).build()));selected="";showProperties();}
    private void duplicate(){var n=design.node(selected);if(n==null)return;var mapping=new HashMap<String,String>();
        for(var c:design.nodes().getValuesAs(JsonObject.class))if(c==n||descendant(c,selected))mapping.put(c.getString("id"),UUID.randomUUID().toString());
        var nodes=Json.createArrayBuilder(design.nodes());for(var c:design.nodes().getValuesAs(JsonObject.class))if(mapping.containsKey(c.getString("id"))){var l=c.getJsonObject("layout");nodes.add(Json.createObjectBuilder(c).add("id",mapping.get(c.getString("id"))).add("name",c.getString("name")+" copy").add("board",mapping.getOrDefault(c.getString("board",""),c.getString("board",""))).add("layout",Json.createObjectBuilder(l).add("x",number(l,"x")+35).add("y",number(l,"y")+35)));}
        var links=Json.createArrayBuilder(design.connections());for(var c:design.connections().getValuesAs(JsonObject.class))if(mapping.containsKey(c.getString("from"))&&mapping.containsKey(c.getString("to")))links.add(Json.createObjectBuilder(c).add("id",UUID.randomUUID().toString()).add("from",mapping.get(c.getString("from"))).add("to",mapping.get(c.getString("to"))));
        selected=mapping.get(selected);edit(new FlightComputerDesign(Json.createObjectBuilder(design.json()).add("nodes",nodes).add("connections",links).build()));}
    private void collapse(){var n=design.node(selected);if(n!=null&&FlightComputerModels.isBoard(n.getString("type"))){var l=n.getJsonObject("layout");replaceNode(Json.createObjectBuilder(n).add("layout",Json.createObjectBuilder(l).add("collapsed",!l.getBoolean("collapsed",false))).build());}}
    private void align(){var n=design.node(selected);if(n==null)return;if(multiSelection.size()<2){JOptionPane.showMessageDialog(this,"Shift-click two or more components to align them.");return;}double y=number(n.getJsonObject("layout"),"y");var b=Json.createArrayBuilder();for(var item:design.nodes().getValuesAs(JsonObject.class))b.add(multiSelection.contains(item.getString("id"))&&!FlightComputerModels.isBoard(item.getString("type"))?Json.createObjectBuilder(item).add("layout",Json.createObjectBuilder(item.getJsonObject("layout")).add("y",y)).build():item);edit(design.with("nodes",b.build()));}
    private static double number(JsonObject o,String key){return o.getJsonNumber(key).doubleValue();}
    private void field(JPanel panel,String label,Component input){
        var row=new JPanel(new BorderLayout(3,3)){
            public Dimension getMaximumSize(){return new Dimension(Integer.MAX_VALUE,getPreferredSize().height);}
        };
        row.setBorder(BorderFactory.createEmptyBorder(5,8,5,8));
        var title=new JLabel(label);title.setFont(title.getFont().deriveFont(Font.BOLD));title.setLabelFor(input);
        row.add(title,BorderLayout.NORTH);row.add(input);row.setAlignmentX(Component.LEFT_ALIGNMENT);panel.add(row);
    }
    private static final class WrappingText extends JTextArea {
        WrappingText(String text){super(text);setEditable(false);setOpaque(false);setLineWrap(true);setWrapStyleWord(true);setFocusable(false);setAlignmentX(Component.LEFT_ALIGNMENT);}
        public Dimension getPreferredSize(){int width=getParent()==null||getParent().getWidth()<120?240:Math.max(100,getParent().getWidth()-16);setSize(width,Short.MAX_VALUE);return super.getPreferredSize();}
    }
    private void enterApplies(Container container,Runnable action){
        for(var child:container.getComponents()){
            if(child instanceof JTextField||child instanceof JTextArea||child instanceof JComboBox<?>)child.addFocusListener(new FocusAdapter(){public void focusGained(FocusEvent e){pendingForm=action;}});
            if(child instanceof JTextField text)text.addActionListener(e->{try{action.run();}catch(Exception ex){error(ex);}});
            else if(child instanceof Container nested)enterApplies(nested,action);
        }
    }
    private void showProperties(){properties.removeAll();applyProperties=()->{};var n=design.node(selected);
        if(n==null){var designName=new JTextField(design.name());designName.setName("fc.designName");field(properties,"Design name",designName);applyProperties=()->{String name=designName.getText().trim();if(name.isEmpty())throw new IllegalArgumentException("Design name is required");edit(design.with("name",Json.createValue(name)));};var info=new WrappingText("Select a board, component or connection.\n\nThe canvas describes architecture; physical mounting is edited explicitly.\n\nDrag components from the library, then connect their named processor ports.");field(properties,"Hardware designer",info);button(properties,"Apply properties",()->applyProperties.run());
            for(var c:design.connections().getValuesAs(JsonObject.class))if(c.getString("id").equals(selected)){field(properties,"Connection",new JLabel(c.getString("port")+" / "+c.getString("bus")));field(properties,"Resource / address",new JLabel(c.getString("resource","")+" / "+c.getString("address","")));button(properties,"Remove connection",this::delete);}
        }else{
            var name=new JTextField(n.getString("name"));name.setName("fc.componentName");field(properties,"Name",name);field(properties,"Model",new JLabel(FlightComputerModels.label(n.getString("type"))));field(properties,"Capability",new WrappingText(FlightComputerModels.capability(n.getString("type"))));
            var parent=new JComboBox<Item>();parent.addItem(new Item("","Outside a board"));for(var b:design.nodes().getValuesAs(JsonObject.class))if(FlightComputerModels.isBoard(b.getString("type"))&&!b.getString("id").equals(selected)&&!descendant(b,selected))parent.addItem(new Item(b.getString("id"),b.getString("name")));
            for(int i=0;i<parent.getItemCount();i++)if(parent.getItemAt(i).id.equals(n.getString("board","")))parent.setSelectedIndex(i);field(properties,"Board",parent);
            var inputs=new LinkedHashMap<String,JSpinner>();for(var spec:FlightComputerModels.properties(n.getString("type"))){var spinner=new JSpinner(new SpinnerNumberModel((n.getJsonObject("properties").containsKey(spec.key())?number(n.getJsonObject("properties"),spec.key()):spec.value()),spec.min(),spec.max(),spec.integer()?1.0:.1));spinner.setEditor(new JSpinner.NumberEditor(spinner,spec.integer()?"0":"0.###"));inputs.put(spec.key(),spinner);field(properties,spec.label(),spinner);}
            var notes=new JTextArea(n.getString("notes",""),4,20);notes.setLineWrap(true);notes.setWrapStyleWord(true);field(properties,"Wiring / model notes",new JScrollPane(notes));
            applyProperties=()->{if(name.getText().isBlank())throw new IllegalArgumentException("Name is required");var props=Json.createObjectBuilder(n.getJsonObject("properties"));inputs.forEach((k,v)->{try{v.commitEdit();}catch(java.text.ParseException ex){throw new IllegalArgumentException(ex);}props.add(k,((Number)v.getValue()).doubleValue());});replaceNode(Json.createObjectBuilder(n).add("name",name.getText()).add("board",((Item)parent.getSelectedItem()).id).add("properties",props).add("notes",notes.getText()).build());};
            button(properties,"Apply properties",()->applyProperties.run());
            if(n.getString("type").equals("java_board"))button(properties,"Edit Java code…",()->editJavaBoard(n.getString("id")));
            if(FlightComputerModels.REQUIRED_PORTS.contains(n.getString("type")))button(properties,"Connect to processor / Java board…",this::connect);
        }
        enterApplies(properties,()->applyProperties.run());properties.revalidate();properties.repaint();
    }
    private void editJavaBoard(String id){
        var node=design.node(id);if(node==null)return;
        var dialog=new JDialog(SwingUtilities.getWindowAncestor(this),"Java board — "+node.getString("name"),Dialog.ModalityType.APPLICATION_MODAL);
        var source=new JTextArea(node.getString("program"));source.setName("fc.javaSource");source.setFont(new Font(Font.MONOSPACED,Font.PLAIN,13));source.setTabSize(4);
        var result=new JTextArea(4,60);result.setEditable(false);result.setLineWrap(true);result.setWrapStyleWord(true);
        var lines=new JTextArea("1");lines.setEditable(false);lines.setFont(source.getFont());lines.setBackground(UIManager.getColor("Panel.background"));
        var initialLines=new StringBuilder();for(int i=1;i<=source.getLineCount();i++)initialLines.append(i).append('\n');lines.setText(initialLines.toString());
        source.getDocument().addDocumentListener(new DocumentListener(){public void insertUpdate(DocumentEvent e){update();}public void removeUpdate(DocumentEvent e){update();}public void changedUpdate(DocumentEvent e){update();}private void update(){var text=new StringBuilder();for(int i=1;i<=source.getLineCount();i++)text.append(i).append('\n');lines.setText(text.toString());result.setText("Source changed. Compile to check it; edited source needs to be enabled again.");}});
        var scroll=new JScrollPane(source);scroll.setRowHeaderView(lines);
        var content=new JPanel(new BorderLayout(6,6));content.setBorder(BorderFactory.createEmptyBorder(8,8,8,8));
        content.add(new WrappingText("Java runs inside OpenRocket with access to this application's files and network. Enable only code you trust; step() must return promptly. Use io.timeUs(), not wall-clock time. Code is stored in the .fc file; execution approval stays on this computer.\n\n"+JavaBoardProgram.MODEL_NOTICE),BorderLayout.NORTH);
        var split=new JSplitPane(JSplitPane.VERTICAL_SPLIT,scroll,new JScrollPane(result));split.setResizeWeight(.8);split.setDividerLocation(420);content.add(split);
        Runnable apply=()->{var current=design.node(id);if(current==null)throw new IllegalArgumentException("Board was removed");replaceNode(Json.createObjectBuilder(current).add("program",source.getText()).build());};
        var buttons=new JPanel(new FlowLayout(FlowLayout.LEFT));
        button(buttons,"Compile",()->{try{JavaBoardProgram.compile(source.getText());result.setText("Compilation successful. "+(JavaBoardProgram.approved(source.getText())?"This source is enabled.":"Use Compile & enable before running this code."));}catch(Exception ex){result.setText(ex.getMessage());}});
        button(buttons,"Compile & enable",()->{try{JavaBoardProgram.compile(source.getText());JavaBoardProgram.approve(source.getText());apply.run();result.setText("Compilation successful. Source applied and enabled on this computer. Save the .fc file to keep the changes.");}catch(Exception ex){result.setText(ex.getMessage());}});
        button(buttons,"Apply source",()->{apply.run();result.setText("Source applied to the design. Save the .fc file to keep the changes.");});
        button(buttons,"API help",()->result.setText("Implement public void step(Context io) in public class BoardProgram extends JavaBoardProgram. Fields persist for this run; each run starts fresh.\nInputs: io.timeUs(), io.dtUs(), io.stateId(), io.connected(port), io.altitudeM(), io.velocityMps(), io.accelerationMps2(), io.signal(name). Signals: "+String.join(", ",FlightComputerDesign.SIGNALS.stream().sorted().toList())+".\nOutputs: io.setAirbrakes(0..1), io.setRollDegrees(-90..90), io.fireRecovery(0..5), io.log(text). Missing outputs discard commands. Recovery/roll are logical only.\nThe embedded compiler supports ordinary Java classes/methods; use explicit types, not var, records or lambdas. Each board runs on its own virtual schedule. timeUs() is the invocation start, dtUs() is the time between starts; measured inputs are frozen at start and outputs appear after execution cost. No inter-board communication delays are modeled. Simultaneous completions use board-ID order; the last write to a shared output wins."));
        button(buttons,"Disable source",()->{JavaBoardProgram.revoke(source.getText());result.setText("Execution disabled for this source on this computer.");});
        var externalButtons=new JPanel(new FlowLayout(FlowLayout.LEFT));var external=new JavaBoardExternalSource[1];
        var externalStatus=new JLabel("External editing uses a working copy; reload it here, then compile, enable and save .fc.");
        buttons.setAlignmentX(Component.LEFT_ALIGNMENT);externalButtons.setAlignmentX(Component.LEFT_ALIGNMENT);externalStatus.setAlignmentX(Component.LEFT_ALIGNMENT);
        var preferences=info.openrocket.core.startup.Application.getPreferences();String editorKey="fc.java.externalEditor";
        button(externalButtons,"Choose external editor…",()->{
            var chooser=new JFileChooser();chooser.setDialogTitle("Choose an editor application or executable");chooser.setFileSelectionMode(JFileChooser.FILES_AND_DIRECTORIES);
            if(chooser.showOpenDialog(dialog)==JFileChooser.APPROVE_OPTION){preferences.putString(editorKey,chooser.getSelectedFile().getAbsolutePath());externalStatus.setText("External editor: "+chooser.getSelectedFile().getName());}
        });
        button(externalButtons,"Use system editor",()->{preferences.putString(editorKey,"");externalStatus.setText("External editing will use the system's editor for Java files.");});
        button(externalButtons,"Edit externally…",()->{try{
            if(external[0]==null)external[0]=new JavaBoardExternalSource(FlightComputerLibrary.directory().resolve("JavaEditing"),source.getText());
            else {
                if(external[0].externallyChanged()&&!external[0].read().equals(source.getText())){
                    JOptionPane.showMessageDialog(dialog,"The working copy changed externally. Use Reload external changes first to preserve those edits.\n"+external[0].file());return;
                }
                external[0].write(source.getText());
            }
            Path path=external[0].file();String editor=preferences.getString(editorKey,"");
            externalStatus.setText(path.toString());externalStatus.setToolTipText(path.toString());
            if(editor.isBlank()){
                if(!Desktop.isDesktopSupported()||!Desktop.getDesktop().isSupported(Desktop.Action.EDIT))throw new IOException("Choose an external editor first. Working copy: "+path);
                Desktop.getDesktop().edit(path.toFile());
            }else if(System.getProperty("os.name","").toLowerCase(java.util.Locale.ROOT).contains("mac")&&editor.toLowerCase(java.util.Locale.ROOT).endsWith(".app"))new ProcessBuilder("/usr/bin/open","-a",editor,path.toString()).redirectOutput(ProcessBuilder.Redirect.DISCARD).redirectError(ProcessBuilder.Redirect.INHERIT).start();
            else new ProcessBuilder(editor,path.toString()).redirectOutput(ProcessBuilder.Redirect.DISCARD).redirectError(ProcessBuilder.Redirect.INHERIT).start();
            result.setText("Opened working copy:\n"+path+"\nSave in your editor, then use Reload external changes here. Compile & enable and save the .fc file to use the edited code.");
        }catch(Exception ex){result.setText("External editor: "+ex.getMessage());}});
        button(externalButtons,"Reload external changes",()->{try{
            if(external[0]==null)throw new IOException("Use Edit externally first to create a working copy.");
            String text=external[0].read();
            if(external[0].editorChanged(source.getText())&&!source.getText().equals(text)&&JOptionPane.showConfirmDialog(dialog,"Replace unsaved edits in this code window with the external working copy?","Reload external source",JOptionPane.OK_CANCEL_OPTION)!=JOptionPane.OK_OPTION)return;
            source.setText(text);external[0].acknowledge(text);result.setText("External source loaded. Compile & enable to check and use this revision, then Save the .fc file.\n"+external[0].file());
        }catch(Exception ex){result.setText("External source: "+ex.getMessage());}});
        Runnable close=()->{if(!source.getText().equals(design.node(id).getString("program"))){int choice=JOptionPane.showConfirmDialog(dialog,"Apply Java source changes to the design?","Java source",JOptionPane.YES_NO_CANCEL_OPTION);if(choice==JOptionPane.CANCEL_OPTION||choice==JOptionPane.CLOSED_OPTION)return;if(choice==JOptionPane.YES_OPTION)apply.run();}dialog.dispose();};
        button(buttons,"Close",close);var footer=new JPanel();footer.setLayout(new BoxLayout(footer,BoxLayout.Y_AXIS));footer.add(buttons);footer.add(externalButtons);footer.add(externalStatus);content.add(footer,BorderLayout.SOUTH);dialog.setContentPane(content);dialog.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);dialog.addWindowListener(new WindowAdapter(){public void windowClosing(WindowEvent e){close.run();}});
        for(String modifier:List.of("control","meta")){source.getInputMap().put(KeyStroke.getKeyStroke(modifier+" S"),"saveSource");source.getInputMap().put(KeyStroke.getKeyStroke(modifier+" ENTER"),"applySource");}
        source.getActionMap().put("saveSource",new AbstractAction(){public void actionPerformed(ActionEvent e){apply.run();save(false);}});source.getActionMap().put("applySource",new AbstractAction(){public void actionPerformed(ActionEvent e){apply.run();}});
        var history=new javax.swing.undo.UndoManager();source.getDocument().addUndoableEditListener(history);
        for(String modifier:List.of("control","meta")){source.getInputMap().put(KeyStroke.getKeyStroke(modifier+" Z"),"undoSource");source.getInputMap().put(KeyStroke.getKeyStroke(modifier+" shift Z"),"redoSource");}
        source.getActionMap().put("undoSource",new AbstractAction(){public void actionPerformed(ActionEvent e){if(history.canUndo())history.undo();}});source.getActionMap().put("redoSource",new AbstractAction(){public void actionPerformed(ActionEvent e){if(history.canRedo())history.redo();}});
        result.setText(JavaBoardProgram.approved(source.getText())?"This source is enabled on this computer.":"Compile & enable to allow this source to run. Loading or editing a file does not execute Java.");
        dialog.setSize(1000,720);dialog.setMinimumSize(new Dimension(800,550));dialog.setLocationRelativeTo(this);dialog.setVisible(true);
    }
    private void connect(){var sources=new JComboBox<Item>();var targets=new JComboBox<Item>();for(var n:design.nodes().getValuesAs(JsonObject.class)){var item=new Item(n.getString("id"),n.getString("name"));if(FlightComputerModels.isControllerHost(n.getString("type")))targets.addItem(item);else if(FlightComputerModels.REQUIRED_PORTS.contains(n.getString("type")))sources.addItem(item);}
        for(int i=0;i<sources.getItemCount();i++)if(sources.getItemAt(i).id.equals(selected))sources.setSelectedIndex(i);
        var bus=new JComboBox<>(new String[]{"SPI","UART","I2C","GPIO","PWM","Power"});var resource=new JTextField("SPI_3");var address=new JTextField();
        var form=new JPanel(new GridLayout(0,2,6,6));form.add(new JLabel("Component"));form.add(sources);form.add(new JLabel("Processor / Java board"));form.add(targets);form.add(new JLabel("Bus / signal"));form.add(bus);form.add(new JLabel("Bus / peripheral resource"));form.add(resource);form.add(new JLabel("Chip select / address / pin"));form.add(address);
        if(JOptionPane.showConfirmDialog(this,form,"Connect hardware",JOptionPane.OK_CANCEL_OPTION)!=JOptionPane.OK_OPTION)return;
        if(sources.getSelectedItem()==null||targets.getSelectedItem()==null)throw new IllegalArgumentException("Add a processor or custom Java board and a supported component first");
        String from=((Item)sources.getSelectedItem()).id,to=((Item)targets.getSelectedItem()).id;String port=design.node(from).getString("type");
        var c=Json.createObjectBuilder().add("id",UUID.randomUUID().toString()).add("from",from).add("to",to).add("port",port).add("bus",(String)bus.getSelectedItem()).add("resource",resource.getText()).add("address",address.getText()).build();selected=c.getString("id");edit(design.with("connections",Json.createArrayBuilder(design.connections()).add(c).build()));
    }
    private void buildBehavior(){
        rules.setName("fc.transitions");
        rules.setCellRenderer(new DefaultListCellRenderer(){public Component getListCellRendererComponent(JList<?> l,Object v,int i,boolean selected,boolean focus){
            var graph=design.stateMachine();var transition=FlightComputerStateMachine.transitions(graph).stream().filter(t->t.getString("id").equals(v)).findFirst().orElse(null);
            String text=v.toString();if(transition!=null){var from=FlightComputerStateMachine.state(graph,transition.getString("from"));var to=FlightComputerStateMachine.state(graph,transition.getString("to"));text=(from==null?"?":from.getString("name"))+" → "+(to==null?"?":to.getString("name"))+" ["+transition.getInt("priority")+"]";}
            return super.getListCellRendererComponent(l,text,i,selected,focus);
        }});
        rules.addListSelectionListener(e->{if(!refreshing&&!e.getValueIsAdjusting()&&rules.getSelectedValue()!=null){ruleId=rules.getSelectedValue();editingState=false;showCondition();}});
        var north=new JPanel(new BorderLayout());var stateScroll=new JScrollPane(stateStrip);stateScroll.setBorder(null);stateScroll.setPreferredSize(new Dimension(700,78));north.add(stateScroll);
        var actions=new JPanel(new FlowLayout(FlowLayout.LEFT));
        button(actions,"Insert state after selected…",this::insertState);button(actions,"Add transition…",this::addTransition);
        var recovery=new JCheckBox("Automatic delayed recovery outputs");recovery.setName("fc.automaticRecovery");
        recovery.setToolTipText("During the recovery phase: channels 3 + 4 after the secondary delay, channel 5 after the tertiary delay. Turn off to control these using your own states.");
        recovery.addActionListener(e->{if(!refreshing)edit(design.with("stateMachine",Json.createObjectBuilder(design.stateMachine()).add("automaticRecovery",recovery.isSelected()).build()));});
        actions.add(recovery);automaticRecovery=recovery;north.add(actions,BorderLayout.SOUTH);behavior.add(north,BorderLayout.NORTH);
        var transitionPanel=new JPanel(new BorderLayout());transitionPanel.add(new JLabel("Transitions (lower priority runs first)"),BorderLayout.NORTH);transitionPanel.add(new JScrollPane(rules));
        var conditions=new JSplitPane(JSplitPane.HORIZONTAL_SPLIT,transitionPanel,conditionHolder);conditions.setDividerLocation(290);conditions.setResizeWeight(.28);
        var parameters=new JPanel();parameters.setLayout(new BoxLayout(parameters,BoxLayout.Y_AXIS));var inputs=new LinkedHashMap<String,JSpinner>();
        for(var spec:FlightComputerModels.PARAMETERS.values()){
            if(spec.key().equals("airbrakeKp"))parameters.add(new WrappingText("Airbrake PID — gains are divided by the existing altitude/area sensitivity K. The original per-update integral and initial deployment schedule are retained. Kd uses error change per virtual second. Defaults: 1, 2, 0."));
            if(spec.key().equals("rollKp"))parameters.add(new WrappingText("Roll PID — error is degrees; Ki integrates degree-seconds and Kd uses measured roll rate. Existing aerodynamic gain scaling and output limits are retained. Defaults: 0.08444, 0, 0.02111."));
            var spinner=new JSpinner(new SpinnerNumberModel(design.parameter(spec.key()),spec.min(),spec.max(),spec.integer()?1.0:spec.key().startsWith("rollK")?.001:.01));
            spinner.setName("fc.parameter."+spec.key());spinner.setEditor(new JSpinner.NumberEditor(spinner,spec.integer()?"0":"0.#########"));inputs.put(spec.key(),spinner);field(parameters,spec.label(),spinner);
        }
        Runnable apply=()->{var values=Json.createObjectBuilder(design.parameters());inputs.forEach((k,v)->{try{v.commitEdit();}catch(java.text.ParseException e){throw new IllegalArgumentException(e);}values.add(k,((Number)v.getValue()).doubleValue());});edit(design.with("parameters",values.build()));};
        button(parameters,"Apply timing / controller settings",apply);enterApplies(parameters,apply);
        parameters.add(new WrappingText("Costs are assumptions unless measured. FC loop 10 ms; PWM 20 ms. Each Java board runs on its own period; its execution cost delays only that board."));
        var split=new JSplitPane(JSplitPane.VERTICAL_SPLIT,conditions,new JScrollPane(parameters));split.setResizeWeight(.65);split.setDividerLocation(340);behavior.add(split);
        parameterInputs=inputs;
    }
    private JCheckBox automaticRecovery;
    private Map<String,JSpinner> parameterInputs=Map.of();
    private void refreshStateStrip(){
        stateStrip.removeAll();var graph=design.stateMachine();
        for(var state:FlightComputerStateMachine.states(graph)){
            var b=new JToggleButton(state.getString("name"));b.setName("fc.state."+state.getString("id"));b.setSelected(state.getString("id").equals(selectedState));
            b.setToolTipText("Phase: "+state.getString("phase")+(state.getString("id").equals(graph.getString("initial"))?" · Initial state":""));
            b.addActionListener(e->{selectedState=state.getString("id");editingState=true;rules.clearSelection();showCondition();});stateStrip.add(b);
        }
        automaticRecovery.setSelected(graph.getBoolean("automaticRecovery",true));
        boolean wasRefreshing=refreshing;refreshing=true;
        rules.setListData(FlightComputerStateMachine.transitions(graph).stream().map(t->t.getString("id")).toArray(String[]::new));
        if(!editingState)rules.setSelectedValue(ruleId,true);refreshing=wasRefreshing;
        stateStrip.revalidate();stateStrip.repaint();
    }
    private void showCondition(){
        if(automaticRecovery==null)return;refreshStateStrip();conditionHolder.removeAll();var graph=design.stateMachine();
        if(editingState){
            var state=FlightComputerStateMachine.state(graph,selectedState);
            if(state!=null)conditionHolder.add(new JScrollPane(stateProperties(state)));
        }else{
            var rule=FlightComputerStateMachine.transitions(graph).stream().filter(r->r.getString("id").equals(ruleId)).findFirst().orElse(null);
            if(rule!=null){
                var panel=new JPanel(new BorderLayout(5,5));
                panel.add(new ConditionEditor(rule.getJsonObject("condition"),value->replaceTransition(Json.createObjectBuilder(rule).add("condition",value).build())));
                var options=new JPanel(new FlowLayout(FlowLayout.LEFT));
                button(options,"Endpoints / priority…",()->editTransition(rule));button(options,"Delete transition",()->removeTransition(rule.getString("id")));panel.add(options,BorderLayout.NORTH);conditionHolder.add(panel);
            }
        }
        parameterInputs.forEach((k,v)->v.setValue(design.parameter(k)));conditionHolder.revalidate();conditionHolder.repaint();
    }
    private JPanel stateProperties(JsonObject state){
        var panel=new PropertyPanel();panel.setLayout(new BoxLayout(panel,BoxLayout.Y_AXIS));var name=new JTextField(state.getString("name"));name.setName("fc.stateName");
        var phase=new JComboBox<>(FlightComputerStateMachine.PHASES.toArray(String[]::new));phase.setSelectedItem(state.getString("phase"));
        field(panel,"State name",name);field(panel,"Flight phase (sensor and telemetry behavior)",phase);
        var initial=new JCheckBox("Initial state",state.getString("id").equals(design.stateMachine().getString("initial")));field(panel,"Startup",initial);
        Runnable apply=()->{if(name.getText().isBlank())throw new IllegalArgumentException("State name is required");var graph=design.stateMachine();if(initial.isSelected())graph=Json.createObjectBuilder(graph).add("initial",state.getString("id")).build();replaceState(graph,Json.createObjectBuilder(state).add("name",name.getText()).add("phase",(String)phase.getSelectedItem()).build());};
        button(panel,"Apply state properties",apply);enterApplies(panel,apply);
        var actions=new JList<>(state.getJsonArray("actions").getValuesAs(JsonObject.class).toArray(JsonObject[]::new));actions.setVisibleRowCount(4);
        actions.setCellRenderer(new DefaultListCellRenderer(){public Component getListCellRendererComponent(JList<?> l,Object v,int i,boolean sel,boolean focus){var a=(JsonObject)v;return super.getListCellRendererComponent(l,FlightComputerStateMachine.ACTIONS.get(a.getString("type"))+(a.containsKey("channel")?" "+a.getInt("channel"):a.containsKey("value")?" "+a.getJsonNumber("value"):a.containsKey("message")?": "+a.getString("message"):""),i,sel,focus);}});
        field(panel,"Actions on entry (run once, in this order)",new JScrollPane(actions));
        var buttons=new JPanel(new FlowLayout(FlowLayout.LEFT));button(buttons,"Add action…",()->editStateAction(state,-1));button(buttons,"Edit action…",()->{if(actions.getSelectedIndex()>=0)editStateAction(state,actions.getSelectedIndex());});
        button(buttons,"Remove action",()->{int index=actions.getSelectedIndex();if(index<0)return;var list=Json.createArrayBuilder();for(int i=0;i<state.getJsonArray("actions").size();i++)if(i!=index)list.add(state.getJsonArray("actions").get(i));replaceState(design.stateMachine(),Json.createObjectBuilder(state).add("actions",list).build());});buttons.setAlignmentX(Component.LEFT_ALIGNMENT);panel.add(buttons);
        button(panel,"Delete state and its transitions",()->{var graph=design.stateMachine();if(graph.getString("initial").equals(state.getString("id")))throw new IllegalArgumentException("Choose another initial state before deleting this one");var nodes=Json.createArrayBuilder();for(var n:FlightComputerStateMachine.states(graph))if(!n.getString("id").equals(selectedState))nodes.add(n);var links=Json.createArrayBuilder();for(var t:FlightComputerStateMachine.transitions(graph))if(!t.getString("from").equals(selectedState)&&!t.getString("to").equals(selectedState))links.add(t);selectedState=graph.getString("initial");edit(design.with("stateMachine",Json.createObjectBuilder(graph).add("states",nodes).add("transitions",links).build()));});
        return panel;
    }
    private void replaceState(JsonObject graph,JsonObject state){var list=Json.createArrayBuilder();for(var n:FlightComputerStateMachine.states(graph))list.add(n.getString("id").equals(state.getString("id"))?state:n);edit(design.with("stateMachine",Json.createObjectBuilder(graph).add("states",list).build()));}
    private void replaceTransition(JsonObject transition){var graph=design.stateMachine();var list=Json.createArrayBuilder();boolean found=false;for(var t:FlightComputerStateMachine.transitions(graph)){boolean match=t.getString("id").equals(transition.getString("id"));found|=match;list.add(match?transition:t);}if(!found)list.add(transition);edit(design.with("stateMachine",Json.createObjectBuilder(graph).add("transitions",list).build()));}
    private void removeTransition(String id){var graph=design.stateMachine();var list=Json.createArrayBuilder();for(var t:FlightComputerStateMachine.transitions(graph))if(!t.getString("id").equals(id))list.add(t);edit(design.with("stateMachine",Json.createObjectBuilder(graph).add("transitions",list).build()));}
    private void insertState(){
        if(FlightComputerStateMachine.state(design.stateMachine(),selectedState)==null)throw new IllegalArgumentException("Select a state first");
        var name=new JTextField("Recovery stage");var delay=new JSpinner(new SpinnerNumberModel(3000,0,600000,100));var fire=new JCheckBox("Fire recovery channel on entry");var channel=new JSpinner(new SpinnerNumberModel(3,0,5,1));
        var panel=new JPanel(new GridLayout(0,2,6,6));panel.add(new JLabel("New state name"));panel.add(name);panel.add(new JLabel("Time in previous state (ms)"));panel.add(delay);panel.add(fire);panel.add(channel);
        if(JOptionPane.showConfirmDialog(this,panel,"Insert intermediate state",JOptionPane.OK_CANCEL_OPTION)!=JOptionPane.OK_OPTION)return;
        try{delay.commitEdit();channel.commitEdit();}catch(java.text.ParseException e){throw new IllegalArgumentException(e);}
        var actions=Json.createArrayBuilder();if(fire.isSelected())actions.add(Json.createObjectBuilder().add("type","fire_recovery").add("channel",((Number)channel.getValue()).intValue()));
        insertIntermediateState(selectedState,name.getText(),((Number)delay.getValue()).doubleValue(),actions.build());
    }
    public void insertIntermediateState(String after,String name,double delayMs,JsonArray actions){
        if(name.isBlank()||!Double.isFinite(delayMs)||delayMs<0)throw new IllegalArgumentException("Enter a name and nonnegative delay");
        String id=UUID.randomUUID().toString();var graph=FlightComputerStateMachine.insert(design.stateMachine(),after,id,name,FlightComputerStateMachine.condition("state_ms",">=",delayMs),actions);selectedState=id;editingState=true;edit(design.with("stateMachine",graph));
    }
    private void addTransition(){
        var graph=design.stateMachine();String from=selectedState;int priority=FlightComputerStateMachine.transitions(graph).stream().filter(t->t.getString("from").equals(from)).mapToInt(t->t.getInt("priority")).max().orElse(-1)+1;
        editTransition(Json.createObjectBuilder().add("id",UUID.randomUUID().toString()).add("from",from).add("to",graph.getString("initial")).add("priority",priority).add("condition",FlightComputerStateMachine.condition("state_ms",">=",1000)).build());
    }
    private void editTransition(JsonObject t){
        var from=new JComboBox<Item>();var to=new JComboBox<Item>();for(var n:FlightComputerStateMachine.states(design.stateMachine())){var item=new Item(n.getString("id"),n.getString("name"));from.addItem(item);to.addItem(item);if(item.id.equals(t.getString("from")))from.setSelectedItem(item);if(item.id.equals(t.getString("to")))to.setSelectedItem(item);}
        var priority=new JSpinner(new SpinnerNumberModel(t.getInt("priority"),0,1000,1));var lockout=new JCheckBox("Require apogee lockout to have elapsed",t.getBoolean("apogeeLockout",false));
        var panel=new JPanel(new GridLayout(0,2,6,6));panel.add(new JLabel("From"));panel.add(from);panel.add(new JLabel("To"));panel.add(to);panel.add(new JLabel("Priority (smaller first)"));panel.add(priority);panel.add(lockout);
        if(JOptionPane.showConfirmDialog(this,panel,"Transition",JOptionPane.OK_CANCEL_OPTION)!=JOptionPane.OK_OPTION)return;
        try{priority.commitEdit();}catch(java.text.ParseException e){throw new IllegalArgumentException(e);}
        ruleId=t.getString("id");editingState=false;replaceTransition(Json.createObjectBuilder(t).add("from",((Item)from.getSelectedItem()).id).add("to",((Item)to.getSelectedItem()).id).add("priority",((Number)priority.getValue()).intValue()).add("apogeeLockout",lockout.isSelected()).build());
    }
    private void editStateAction(JsonObject state,int index){
        var previous=index<0?Json.createObjectBuilder().add("type","fire_recovery").add("channel",0).build():state.getJsonArray("actions").getJsonObject(index);
        var type=new JComboBox<Item>();FlightComputerStateMachine.ACTIONS.forEach((id,label)->{var item=new Item(id,label);type.addItem(item);if(id.equals(previous.getString("type")))type.setSelectedItem(item);});
        var value=new JTextField(previous.containsKey("channel")?String.valueOf(previous.getInt("channel")):previous.containsKey("value")?previous.getJsonNumber("value").toString():previous.getString("message",""));
        var panel=new JPanel(new GridLayout(0,1,6,6));panel.add(type);panel.add(new JLabel("Channel, output value or message (if needed):"));panel.add(value);
        if(JOptionPane.showConfirmDialog(this,panel,"Entry action",JOptionPane.OK_CANCEL_OPTION)!=JOptionPane.OK_OPTION)return;
        String key=((Item)type.getSelectedItem()).id;var action=Json.createObjectBuilder().add("type",key);switch(key){case "fire_recovery"->action.add("channel",Integer.parseInt(value.getText().trim()));case "airbrakes","roll"->action.add("value",Double.parseDouble(value.getText().trim()));case "log"->action.add("message",value.getText());}
        var a=action.build();FlightComputerStateMachine.validateAction(a);var list=Json.createArrayBuilder();for(int i=0;i<state.getJsonArray("actions").size();i++)list.add(i==index?a:state.getJsonArray("actions").get(i));if(index<0)list.add(a);replaceState(design.stateMachine(),Json.createObjectBuilder(state).add("actions",list).build());
    }
    private final class ConditionEditor extends JPanel {
        private final JTree conditionTree;private final Consumer<JsonObject> save;
        ConditionEditor(JsonObject root,Consumer<JsonObject> save){super(new BorderLayout(5,5));this.save=save;
            conditionTree=new JTree(build(root));for(int i=0;i<conditionTree.getRowCount();i++)conditionTree.expandRow(i);conditionTree.setSelectionRow(0);add(new JScrollPane(conditionTree));
            var buttons=new JPanel(new FlowLayout(FlowLayout.LEFT));button(buttons,"Edit…",()->change(false,false));button(buttons,"Add condition…",()->change(true,false));button(buttons,"Add group…",()->change(true,true));button(buttons,"Remove",()->{var node=selection();if(node.getParent()!=null){((DefaultMutableTreeNode)node.getParent()).remove(node);commit();}});add(buttons,BorderLayout.SOUTH);
            add(new WrappingText("Signals: acceleration m/s², altitude/drop m, velocity m/s, timers ms. state_ms resets on entry; command uses the telemetry phase ID."),BorderLayout.NORTH);
        }
        private record Term(JsonObject value){public String toString(){String op=value.getString("op");return op.equals("all")?"ALL conditions":op.equals("any")?"ANY condition":value.getString("signal")+" "+op+" "+value.getJsonNumber("value");}}
        private DefaultMutableTreeNode build(JsonObject c){var n=new DefaultMutableTreeNode(new Term(c));if(c.containsKey("terms"))for(var t:c.getJsonArray("terms").getValuesAs(JsonObject.class))n.add(build(t));return n;}
        private DefaultMutableTreeNode selection(){return (DefaultMutableTreeNode)conditionTree.getLastSelectedPathComponent();}
        private JsonObject serialize(DefaultMutableTreeNode n){var c=((Term)n.getUserObject()).value;if(c.containsKey("terms")){var terms=Json.createArrayBuilder();for(int i=0;i<n.getChildCount();i++)terms.add(serialize((DefaultMutableTreeNode)n.getChildAt(i)));return Json.createObjectBuilder(c).add("terms",terms).build();}return c;}
        private void commit(){save.accept(serialize((DefaultMutableTreeNode)conditionTree.getModel().getRoot()));}
        private void change(boolean add,boolean group){var n=selection();if(n==null)return;JsonObject prior=((Term)n.getUserObject()).value;
            if(add&&!prior.containsKey("terms")){JOptionPane.showMessageDialog(this,"Select an ALL or ANY group to add a condition.");return;}
            boolean isGroup=group||(!add&&prior.containsKey("terms"));JsonObject next;
            if(isGroup){var options=new JComboBox<>(new String[]{"all","any"});if(!add)options.setSelectedItem(prior.getString("op"));if(JOptionPane.showConfirmDialog(this,options,"Condition group",JOptionPane.OK_CANCEL_OPTION)!=JOptionPane.OK_OPTION)return;next=Json.createObjectBuilder().add("op",(String)options.getSelectedItem()).add("terms",add?Json.createArrayBuilder().build():prior.getJsonArray("terms")).build();}
            else{var signals=new JComboBox<>(FlightComputerDesign.SIGNALS.stream().sorted().toArray(String[]::new));var op=new JComboBox<>(new String[]{">",">=","<","<=","==","!="});var value=new JSpinner(new SpinnerNumberModel(add?0:prior.getJsonNumber("value").doubleValue(),-1e9,1e9,1.0));if(!add){signals.setSelectedItem(prior.getString("signal"));op.setSelectedItem(prior.getString("op"));}var form=new JPanel(new GridLayout(0,2,6,6));form.add(new JLabel("Signal"));form.add(signals);form.add(new JLabel("Comparison"));form.add(op);form.add(new JLabel("Value"));form.add(value);if(JOptionPane.showConfirmDialog(this,form,"Condition",JOptionPane.OK_CANCEL_OPTION)!=JOptionPane.OK_OPTION)return;try{value.commitEdit();}catch(Exception e){error(e);return;}next=Json.createObjectBuilder().add("signal",(String)signals.getSelectedItem()).add("op",(String)op.getSelectedItem()).add("value",((Number)value.getValue()).doubleValue()).build();}
            if(add)n.add(build(next));else n.setUserObject(new Term(next));commit();
        }
    }
    private JPanel buildTests(){var panel=new JPanel(new BorderLayout(5,5));var controls=new JPanel(new FlowLayout(FlowLayout.LEFT));
        button(controls,"Pad test (3 s)",()->bench(null));button(controls,"Replay raw sensor CSV…",()->{var chooser=new JFileChooser();if(chooser.showOpenDialog(this)==JFileChooser.APPROVE_OPTION)bench(chooser.getSelectedFile().toPath());});
        button(controls,"Run saved design on this rocket",this::fullFlight);button(controls,"Export trace…",()->{var chooser=new JFileChooser();if(chooser.showSaveDialog(this)==JFileChooser.APPROVE_OPTION)try{Files.writeString(chooser.getSelectedFile().toPath(),testLog.getText());}catch(Exception e){error(e);}});
        var quantity=new JComboBox<>(new String[]{"Barometric altitude (m)","Integrated velocity (m/s)","Airbrake fraction","Flight state code","Sample age (µs)"});quantity.addActionListener(e->{tracePlot.quantity=quantity.getSelectedIndex();tracePlot.repaint();});controls.add(quantity);var testHeader=new JPanel(new BorderLayout());testHeader.add(controls,BorderLayout.NORTH);testHeader.add(new WrappingText(JavaBoardProgram.MODEL_NOTICE),BorderLayout.SOUTH);panel.add(testHeader,BorderLayout.NORTH);
        testLog.setEditable(false);testLog.setFont(new Font(Font.MONOSPACED,Font.PLAIN,11));var split=new JSplitPane(JSplitPane.VERTICAL_SPLIT,tracePlot,new JScrollPane(testLog));split.setResizeWeight(.5);split.setDividerLocation(270);panel.add(split);
        var instructions=new JTextArea("Replay requires raw engineering-unit samples: time_us,ax,ay,az,pressure_hpa,temperature_c,gx,gy,gz,latitude,longitude,gps_altitude_m. First timestamp must be 0; later timestamps strictly increase. Acceleration is in sensor axes (m/s²), gyro in deg/s. Inputs are held between rows. This is not a GS telemetry decoder. Bench tests use the same virtual-time dispatcher as flights; recovery/roll actions remain recorded only.");instructions.setLineWrap(true);instructions.setWrapStyleWord(true);instructions.setEditable(false);instructions.setRows(3);panel.add(instructions,BorderLayout.SOUTH);return panel;
    }
    private void bench(Path replay){FlightComputerDesign snapshot=design;snapshot.requireRunnable();testLog.setText("Scratch test of "+snapshot.name()+" · "+(dirty()?"unsaved design":"saved design")+"\n");
        new SwingWorker<info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener.BenchResult,String>(){
            protected info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener.BenchResult doInBackground() throws Exception {
                java.util.function.LongFunction<edu.mit.rocket_team.zephyrus.FC.RTFC.Inputs> samples;long end;
                if(replay==null){end=3_000_000;samples=now->new edu.mit.rocket_team.zephyrus.FC.RTFC.Inputs(now,new edu.mit.rocket_team.zephyrus.util.data.RTAccelData(9.8065f,0,0),new edu.mit.rocket_team.zephyrus.util.data.RTBaroData(Float.NaN,Float.NaN,20,Float.NaN,1013.25f),new edu.mit.rocket_team.zephyrus.util.data.RTGPSData(0.0,0.0,0.0,0.0,0.0,0.0,true),new edu.mit.rocket_team.zephyrus.util.data.RTGyroData(0f,0f,0f));}
                else{var data=FlightComputerReplay.read(replay);end=data.lastKey();samples=now->data.floorEntry(now).getValue();}
                return info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener.bench(snapshot,end,samples,line->{System.out.println(line);publish(line);});
            }
            protected void process(List<String> lines){appendLog(lines);}
            protected void done(){try{var result=get();tracePlot.points=result.points();tracePlot.repaint();testLog.append("\n"+result.timing()+"\nCSV: "+result.csv()+"\nLog: "+result.log()+"\n");}catch(Exception e){error(e);}}
        }.execute();
    }
    private void appendLog(List<String> lines){for(String line:lines)testLog.append(line+"\n");if(testLog.getDocument().getLength()>1_000_000)try{testLog.getDocument().remove(0,testLog.getDocument().getLength()-800_000);}catch(Exception ignored){}}
    private void fullFlight(){if(simulation==null)throw new IllegalArgumentException("Open the designer from a simulation to run this rocket");if(dirty()&&!save(false))return;design.requireRunnable();
        var copy=simulation.copy();try{ZephyrusFlightComputer.selectFile(copy,file);}catch(IOException e){error(e);return;}
        var fc=ZephyrusFlightComputer.read(copy);ZephyrusFlightComputer.apply(copy,true,fc.getLinkSettings(),edu.mit.rocket_team.zephyrus.telemetry.FlightComputerOutputSettings.DEFAULT,fc.getTimingSettings());copy.getOptions().setEnsembleSettings(copy.getOptions().getEnsembleSettings().disabled());
        testLog.setText("Running a single test flight of the saved design. Results do not replace the simulation's existing results.\n");
        new SwingWorker<Void,String>(){protected Void doInBackground()throws Exception{copy.simulate();return null;}protected void done(){try{get();var p=copy.getFlightComputerLogPath();if(p!=null){String content=Files.readString(p);testLog.append(content.length()>800000?content.substring(content.length()-800000):content);}testLog.append("\nFull log: "+p+"\nTelemetry: "+copy.getFlightComputerTelemetryPath());}catch(Exception e){error(e);}}}.execute();
    }
    private final class TracePlot extends JPanel {
        List<info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener.BenchPoint> points=List.of();int quantity;
        TracePlot(){setPreferredSize(new Dimension(600,270));setToolTipText("Time-aligned FC trace");}
        double value(info.openrocket.core.simulation.listeners.FlightControllerSimulatorListener.BenchPoint p){return switch(quantity){case 0->p.altitude();case 1->p.velocity();case 2->p.output();case 3->p.state();default->p.sampleAgeUs();};}
        protected void paintComponent(Graphics raw){super.paintComponent(raw);var g=(Graphics2D)raw.create();g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);int w=getWidth()-90,h=getHeight()-55;g.setColor(getForeground());g.drawLine(65,15,65,h+15);g.drawLine(65,h+15,w+65,h+15);g.drawString("Time (s)",w/2,h+45);
            if(!points.isEmpty()){double lo=points.stream().mapToDouble(this::value).min().orElse(0),hi=points.stream().mapToDouble(this::value).max().orElse(1);if(hi==lo)hi=lo+1;double end=Math.max(.001,points.get(points.size()-1).seconds());g.drawString(String.format(Locale.ROOT,"%.3g",hi),5,25);g.drawString(String.format(Locale.ROOT,"%.3g",lo),5,h+15);g.drawString(String.format(Locale.ROOT,"%.2f",end),w+40,h+32);var line=new Path2D.Double();for(int i=0;i<points.size();i++){var p=points.get(i);double x=65+p.seconds()/end*w,y=15+h-(value(p)-lo)/(hi-lo)*h;if(i==0)line.moveTo(x,y);else line.lineTo(x,y);}g.setColor(new Color(22,152,192));g.setStroke(new BasicStroke(2));g.draw(line);}else g.drawString("Run a bench test or replay to inspect its trace",90,55);g.dispose();}
    }
        private static final class PropertyPanel extends JPanel implements Scrollable {
        public Dimension getPreferredScrollableViewportSize(){return new Dimension(280,450);}
        public int getScrollableUnitIncrement(Rectangle r,int orientation,int direction){return 22;}
        public int getScrollableBlockIncrement(Rectangle r,int orientation,int direction){return r.height-22;}
        public boolean getScrollableTracksViewportWidth(){return true;}
        public boolean getScrollableTracksViewportHeight(){return false;}
    }
    public void fitView(){canvas.fit();}
    private final class Canvas extends JPanel {
        private double scale=.85,panX=15,panY=15;private Point lastMouse,press;private String dragging="";private FlightComputerDesign dragStart;
        private final Map<String,Shape> links=new HashMap<>();
        private final Map<String,Rectangle2D> linkLabels=new LinkedHashMap<>();
        Canvas(){addComponentListener(new ComponentAdapter(){private boolean fitted;public void componentResized(ComponentEvent e){if(!fitted&&getWidth()>0&&getHeight()>0){fit();fitted=true;}}});setBackground(new Color(24,33,45));setName("fc.designer.canvas");setFocusable(true);
            var mouse=new MouseAdapter(){
                public void mousePressed(MouseEvent e){requestFocusInWindow();lastMouse=e.getPoint();press=e.getPoint();dragStart=design;
                    if(SwingUtilities.isMiddleMouseButton(e)||e.isAltDown()){dragging="@pan";return;}
                    var point=world(e.getPoint());String id=hit(point);
                    if(e.isShiftDown()){if(!multiSelection.add(id))multiSelection.remove(id);selected=id;showProperties();repaint();}
                    else if(!multiSelection.contains(id))select(id);else{selected=id;showProperties();}
                    dragging=design.node(id)==null?"":id;
                    if(e.getClickCount()==2&&!dragging.isEmpty())collapse();
                }
                public void mouseDragged(MouseEvent e){if(lastMouse==null)return;double dx=(e.getX()-lastMouse.x)/scale,dy=(e.getY()-lastMouse.y)/scale;
                    if(dragging.equals("@pan")){panX+=e.getX()-lastMouse.x;panY+=e.getY()-lastMouse.y;}
                    else if(!dragging.isEmpty()){var b=Json.createArrayBuilder();for(var n:design.nodes().getValuesAs(JsonObject.class)){if(multiSelection.contains(n.getString("id"))||selectedAncestor(n)){var l=n.getJsonObject("layout");n=Json.createObjectBuilder(n).add("layout",Json.createObjectBuilder(l).add("x",number(l,"x")+dx).add("y",number(l,"y")+dy)).build();}b.add(n);}design=design.with("nodes",b.build());}
                    lastMouse=e.getPoint();repaint();
                }
                public void mouseReleased(MouseEvent e){if(dragStart!=null&&!design.json().equals(dragStart.json())){undo.push(dragStart);redo.clear();refresh();}lastMouse=null;dragStart=null;dragging="";}
                public void mouseWheelMoved(MouseWheelEvent e){var before=world(e.getPoint());scale=Math.max(.25,Math.min(2.5,scale*Math.pow(1.12,-e.getPreciseWheelRotation())));panX=e.getX()-before.getX()*scale;panY=e.getY()-before.getY()*scale;repaint();}
            };addMouseListener(mouse);addMouseMotionListener(mouse);addMouseWheelListener(mouse);
            getInputMap(WHEN_FOCUSED).put(KeyStroke.getKeyStroke("DELETE"),"delete");getActionMap().put("delete",new AbstractAction(){public void actionPerformed(ActionEvent e){delete();}});
            setTransferHandler(new TransferHandler(){public boolean canImport(TransferSupport s){return s.isDataFlavorSupported(DataFlavor.stringFlavor);}public boolean importData(TransferSupport s){try{String type=(String)s.getTransferable().getTransferData(DataFlavor.stringFlavor);if(!FlightComputerModels.TYPES.contains(type))return false;var p=world(s.getDropLocation().getDropPoint());addNode(type,p.getX(),p.getY());return true;}catch(Exception e){error(e);return false;}}});
        }
        private boolean selectedAncestor(JsonObject n){for(String id:multiSelection)if(descendant(n,id))return true;return false;}
        private Point2D world(Point p){return new Point2D.Double((p.x-panX)/scale,(p.y-panY)/scale);}
        private boolean visible(JsonObject n){var visited=new HashSet<String>();String parent=n.getString("board","");while(!parent.isEmpty()&&visited.add(parent)){var b=design.node(parent);if(b==null)return true;if(b.getJsonObject("layout").getBoolean("collapsed",false))return false;parent=b.getString("board","");}return true;}
        private String visibleId(String id){var n=design.node(id);var visited=new HashSet<String>();String result=id;while(n!=null&&!n.getString("board","").isEmpty()&&visited.add(n.getString("id"))){n=design.node(n.getString("board"));if(n!=null&&n.getJsonObject("layout").getBoolean("collapsed",false))result=n.getString("id");}return result;}
        private Rectangle2D bounds(JsonObject n){var l=n.getJsonObject("layout");double x=number(l,"x"),y=number(l,"y");if(!FlightComputerModels.isBoard(n.getString("type"))||l.getBoolean("collapsed",false))return new Rectangle2D.Double(x,y,205,65);
            var b=new Rectangle2D.Double(x,y,240,125);for(var child:design.nodes().getValuesAs(JsonObject.class))if(descendant(child,n.getString("id"))){var cl=child.getJsonObject("layout");b.add(new Rectangle2D.Double(number(cl,"x")-12,number(cl,"y")-12,230,90));}return b;}
        private String hit(Point2D p){var all=design.nodes().getValuesAs(JsonObject.class);for(int i=all.size()-1;i>=0;i--){var n=all.get(i);if(visible(n)&&!FlightComputerModels.isBoard(n.getString("type"))&&bounds(n).contains(p))return n.getString("id");}
            for(var entry:linkLabels.entrySet())if(entry.getValue().contains(p))return entry.getKey();
            for(var entry:links.entrySet())if(new BasicStroke((float)(10/scale)).createStrokedShape(entry.getValue()).contains(p))return entry.getKey();
            for(int i=all.size()-1;i>=0;i--){var n=all.get(i);if(visible(n)&&bounds(n).contains(p))return n.getString("id");}return "";}
        void fit(){Rectangle2D union=null;for(var n:design.nodes().getValuesAs(JsonObject.class))if(visible(n)){var b=bounds(n);if(union==null)union=(Rectangle2D)b.clone();else union.add(b);}if(union==null)return;scale=Math.max(.25,Math.min(1.6,Math.min((getWidth()-40)/union.getWidth(),(getHeight()-40)/union.getHeight())));panX=20-union.getX()*scale;panY=20-union.getY()*scale;repaint();}
        protected void paintComponent(Graphics raw){super.paintComponent(raw);var g=(Graphics2D)raw.create();g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);g.translate(panX,panY);g.scale(scale,scale);
            for(var n:design.nodes().getValuesAs(JsonObject.class))if(visible(n)&&FlightComputerModels.isBoard(n.getString("type"))){var b=bounds(n);g.setColor(new Color(35,48,63));g.fill(new RoundRectangle2D.Double(b.getX(),b.getY(),b.getWidth(),b.getHeight(),16,16));g.setColor(multiSelection.contains(n.getString("id"))?new Color(92,206,255):new Color(78,105,125));g.setStroke(new BasicStroke(2));g.draw(b);g.setFont(getFont().deriveFont(Font.BOLD,14));g.setColor(new Color(172,203,222));g.drawString((n.getJsonObject("layout").getBoolean("collapsed",false)?"▸ ":"▾ ")+n.getString("name"),(float)b.getX()+10,(float)b.getY()+22);}
            links.clear();g.setFont(getFont().deriveFont(11f));for(var c:design.connections().getValuesAs(JsonObject.class)){var a=design.node(visibleId(c.getString("from")));var b=design.node(visibleId(c.getString("to")));if(a==null||b==null||a==b)continue;var aa=bounds(a);var bb=bounds(b);double x1=aa.getCenterX(),y1=aa.getCenterY(),x2=bb.getCenterX(),y2=bb.getCenterY();var path=new CubicCurve2D.Double(x1,y1,x1+50,y1,x2-50,y2,x2,y2);links.put(c.getString("id"),path);g.setColor(c.getString("id").equals(selected)?Color.YELLOW:new Color(92,173,189));g.setStroke(new BasicStroke(c.getString("id").equals(selected)?3:1.3f));g.draw(path);}
            for(var n:design.nodes().getValuesAs(JsonObject.class))if(visible(n)&&!FlightComputerModels.isBoard(n.getString("type"))){var b=bounds(n);g.setColor(multiSelection.contains(n.getString("id"))?new Color(31,103,139):new Color(46,66,84));g.fill(new RoundRectangle2D.Double(b.getX(),b.getY(),b.getWidth(),b.getHeight(),12,12));g.setColor(multiSelection.contains(n.getString("id"))?new Color(120,222,255):new Color(101,139,164));g.setStroke(new BasicStroke(1.5f));g.draw(new RoundRectangle2D.Double(b.getX(),b.getY(),b.getWidth(),b.getHeight(),12,12));g.setColor(Color.WHITE);g.setFont(getFont().deriveFont(Font.BOLD,13));g.drawString(n.getString("name"),(float)b.getX()+10,(float)b.getY()+23);g.setFont(getFont().deriveFont(11f));g.setColor(new Color(174,206,224));g.drawString(n.getString("type"),(float)b.getX()+10,(float)b.getY()+43);g.fill(new Ellipse2D.Double(b.getX()-4,b.getCenterY()-4,8,8));g.fill(new Ellipse2D.Double(b.getMaxX()-4,b.getCenterY()-4,8,8));}
            paintConnectionLabels(g);
            g.dispose();
        }
        private Point2D curvePoint(CubicCurve2D c,double t){double u=1-t;return new Point2D.Double(u*u*u*c.getX1()+3*u*u*t*c.getCtrlX1()+3*u*t*t*c.getCtrlX2()+t*t*t*c.getX2(),u*u*u*c.getY1()+3*u*u*t*c.getCtrlY1()+3*u*t*t*c.getCtrlY2()+t*t*t*c.getY2());}
        private void paintConnectionLabels(Graphics2D g){
            linkLabels.clear();g.setFont(getFont().deriveFont(11f));var fm=g.getFontMetrics();
            var obstacles=new ArrayList<Rectangle2D>();
            for(var n:design.nodes().getValuesAs(JsonObject.class))if(visible(n)){
                var b=bounds(n);boolean board=FlightComputerModels.isBoard(n.getString("type"))&&!n.getJsonObject("layout").getBoolean("collapsed",false);
                obstacles.add(new Rectangle2D.Double(b.getX()-3,b.getY()-3,b.getWidth()+6,board?30:b.getHeight()+6));
            }
            for(var c:design.connections().getValuesAs(JsonObject.class)){
                if(!(links.get(c.getString("id")) instanceof CubicCurve2D curve))continue;
                var a=bounds(design.node(visibleId(c.getString("from"))));var b=bounds(design.node(visibleId(c.getString("to"))));
                double start=0,end=1;
                for(int i=0;i<=100;i++){double t=i/100.0;if(!a.contains(curvePoint(curve,t))){start=t;break;}}
                for(int i=100;i>=0;i--){double t=i/100.0;if(!b.contains(curvePoint(curve,t))){end=t;break;}}
                // Connections store component -> processor; visually label toward the component.
                var anchor=curvePoint(curve,start+(end-start)*.25);
                String text=c.getString("bus")+" · "+c.getString("port");double w=fm.stringWidth(text)+8,h=fm.getHeight()+4;
                Rectangle2D label=null;double best=Double.POSITIVE_INFINITY;
                for(int dy=-5;dy>=-110;dy-=h+3)for(double dx:new double[]{-w/2,4,-w-4}){
                    var candidate=new Rectangle2D.Double(anchor.getX()+dx,anchor.getY()+dy-h,w,h);
                    boolean blocked=obstacles.stream().anyMatch(candidate::intersects)||linkLabels.values().stream().anyMatch(candidate::intersects);
                    double score=blocked?1e9:Math.abs(dy)+Math.abs(dx+w/2)*.4;
                    if(score<best){label=candidate;best=score;}
                }
                linkLabels.put(c.getString("id"),label);
                g.setColor(c.getString("id").equals(selected)?Color.YELLOW:new Color(122,207,221));g.setStroke(new BasicStroke(.7f));
                if(label.getMaxY()<anchor.getY()-12)g.draw(new Line2D.Double(anchor.getX(),anchor.getY(),label.getCenterX(),label.getMaxY()));
                g.setColor(new Color(24,33,45,235));g.fill(new RoundRectangle2D.Double(label.getX(),label.getY(),w,h,5,5));
                g.setColor(c.getString("id").equals(selected)?Color.YELLOW:new Color(122,207,221));g.drawString(text,(float)label.getX()+4,(float)label.getY()+2+fm.getAscent());
            }
        }
    }
}
