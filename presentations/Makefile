DECKS := $(wildcard */slides.md)
HTML  := $(DECKS:slides.md=slides.html)
PDF   := $(DECKS:slides.md=slides.pdf)

.PHONY: html pdf clean help

html: $(HTML)
pdf:  $(PDF)

%/slides.html: %/slides.md %/theme-ccom.css
	marp --theme $*/theme-ccom.css $< -o $@

%/slides.pdf: %/slides.md %/theme-ccom.css
	marp --theme $*/theme-ccom.css $< --pdf -o $@

clean:
	rm -f $(HTML) $(PDF)

help:
	@echo "Targets: html, pdf, clean"
	@echo "Decks:   $(dir $(DECKS))"
