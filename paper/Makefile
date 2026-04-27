PAPER = main

all: $(PAPER).pdf

$(PAPER).pdf: $(PAPER).tex sections/*.tex references.bib
	pdflatex $(PAPER).tex
	bibtex $(PAPER)
	pdflatex $(PAPER).tex
	pdflatex $(PAPER).tex

clean:
	rm -f *.aux *.bbl *.blg *.log *.out *.toc *.fdb_latexmk *.fls *.synctex.gz

distclean: clean
	rm -f $(PAPER).pdf

.PHONY: all clean distclean
