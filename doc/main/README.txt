OpenRocket MIT application architecture and developer guide

Open main.tex, or open this folder as a LaTeX project with main.tex as the
main document. Run ./build.sh with latexmk and a LaTeX distribution installed
to produce main.pdf. Auxiliary files go in .build/.

The guide has 14 chapters, a source-navigation appendix, and a whole-codebase
package atlas. It describes inherited OpenRocket functionality as well as MIT
additions, including every native component family and the implemented drag,
lift, mass, environment, and recovery models. Each chapter is
editable independently under sections/. Layout and shared diagram/link macros
are in preamble.tex. Topic supplements are included from the numbered chapters.
source-inventory.tsv maps every main Java source file to a subsystem and chapter.
Regenerate it and the package atlas with python3 tools/build_source_inventory.py.
The inventory covers 1,282 files in 139 named packages plus module descriptors. All diagrams are vector TikZ; no external assets or
shell-escape are needed.

The original flight-computer architecture TeX and PDF are copied unchanged
under reference/. Chapter 8 incorporates the report and clarifies the current
engine default, step-limiting hook, and controller/Java-task state behavior.
The original historical application test results remain identified as such.

Documented source: 7f0b67ee1c755ec89f154b0f0d2f4e23c843aa87
Published v6.4:   9f71a94196a6284ae2b574639aafbb2bf1bc175a
Prepared:        27 October 2026

Source links are pinned to the documented commit. If updating the source
edition, update the title page, preamble link macros, release comparison,
and affected behavioral descriptions together.
