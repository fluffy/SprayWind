

SRC  := $(wildcard *.md)
PNG  := $(wildcard *.png)

HTML := $(patsubst %.md,%.html,$(SRC))
PDF  := $(patsubst %.md,%.pdf,$(SRC))


all: $(HTML) $(PDF)

clean:
	rm -f $(HTML) $(PDF)


%.html: %.md $(PNG)
	pandoc -t html5 $< -o $@

%.pdf: %.html $(PNG)
	wkpdf -p letter -s $< -o $@
