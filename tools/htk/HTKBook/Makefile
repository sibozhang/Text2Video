all: htkbook.ps html htkbook.pdf

maintex =    htkbook.tex HTK.tex errors.tex hlmtutorial.tex labels.tex      \
	     speechio.tex adapt.tex exampsys.tex models.tex                 \
	     train.tex config.tex fundaments.tex netdict.tex decode.tex     \
	     hlmfiles.tex htkoview.tex openviron.tex discmods.tex           \
	     hlmfund.tex htkslf.tex refine.tex

reftex =     HTKRef/Cluster.tex  HTKRef/HInit.tex    HTKRef/HResults.tex    \
	     HTKRef/LGCopy.tex	 HTKRef/HBuild.tex   HTKRef/HLEd.tex        \
	     HTKRef/HSGen.tex    HTKRef/LGList.tex   HTKRef/HCompV.tex      \
	     HTKRef/HLMCopy.tex  HTKRef/HSLab.tex    HTKRef/LGPrep.tex      \
	     HTKRef/HCopy.tex    HTKRef/HLStats.tex  HTKRef/HSmooth.tex     \
	     HTKRef/LMerge.tex   HTKRef/HDMan.tex    HTKRef/HList.tex       \
             HTKRef/HVite.tex    HTKRef/LPlex.tex                           \
	     HTKRef/HParse.tex   HTKRef/LAdapt.tex   HTKRef/LSubset.tex     \
	     HTKRef/HERest.tex   HTKRef/HQuant.tex   HTKRef/LBuild.tex      \
	     HTKRef/tools.tex    HTKRef/HHEd.tex     HTKRef/HRest.tex       \
	     HTKRef/LFoF.tex     HTKRef/HLRescore.tex

figures =    HTKFigs/Blocking.eps      HTKFigs/melfbank.eps		    \
	     HTKFigs/Config.eps        HTKFigs/memheap.eps		    \
	     HTKFigs/HLMoperation.eps  HTKFigs/messencode.eps		    \
	     HTKFigs/HTKFormat.eps     HTKFigs/mononet.eps		    \
	     HTKFigs/LTool.eps         HTKFigs/muacc.eps		    \
	     HTKFigs/netforcsr.eps     HTKFigs/overview.eps		    \
	     HTKFigs/MMFeg.eps         HTKFigs/parher.eps		    \
	     HTKFigs/ClassLM.eps       HTKFigs/qstree.eps		    \
	     HTKFigs/Spmods.eps        HTKFigs/recipe.eps		    \
	     HTKFigs/Tool.decode.eps   HTKFigs/recnetlev.eps		    \
	     HTKFigs/digitnets.eps     HTKFigs/subsmixrep.eps		    \
	     HTKFigs/dmaker.eps        HTKFigs/subword.eps		    \
	     HTKFigs/egsils.eps        HTKFigs/sysoview.eps		    \
	     HTKFigs/egtranstie.eps    HTKFigs/tcover.eps		    \
	     HTKFigs/endpointer.eps    HTKFigs/tiedstate.eps		    \
	     HTKFigs/flatst.eps        HTKFigs/tmixeg.eps		    \
	     HTKFigs/headapt.eps       HTKFigs/toolkit.eps		    \
	     HTKFigs/headaptrdp.eps    HTKFigs/tracc.eps		    \
	     HTKFigs/herestdp.eps      HTKFigs/tsubword.eps		    \
	     HTKFigs/hierarch.eps      HTKFigs/useforiso.eps		    \
	     HTKFigs/hinitdp.eps       HTKFigs/vaacc.eps		    \
	     HTKFigs/hmm1.eps          HTKFigs/vitloop.eps		    \
	     HTKFigs/hsetdef.eps       HTKFigs/vmatrep.eps		    \
	     HTKFigs/hslab.eps         HTKFigs/vqtohmm.eps		    \
	     HTKFigs/htktools.eps      HTKFigs/vtlnpiecewise.eps	    \
	     HTKFigs/hvalign.eps       HTKFigs/vtrellis.eps		    \
	     HTKFigs/isoprob.eps       HTKFigs/wdnet.eps		    \
	     HTKFigs/isoword.eps       HTKFigs/wdnet0.eps		    \
	     HTKFigs/itemtree.eps      HTKFigs/wdnet1.eps		    \
	     HTKFigs/labegs.eps        HTKFigs/wintnet.eps		    \
	     HTKFigs/WordLM.eps        HTKFigs/wtacc.eps		    \
	     HTKFigs/markovgen.eps     HTKFigs/xwrdnet.eps                  \
             HTKFigs/wlroper.eps

htkbook.pdf: $(maintex) $(reftex) $(figures)
	sed -e 's/\%\\usepackage\[dvipdfm/\\usepackage\[dvipdfm/' htkbook.tex >htkbook-pdf.tex
	latex htkbook-pdf.tex
	latex htkbook-pdf.tex
	makeindex htkbook-pdf.idx
	latex htkbook-pdf.tex
	dvipdfm -p a4 -o htkbook.pdf htkbook-pdf.dvi
	# If dvipdfm is not installed then try:
	# dvipdf htkbook-pdf.dvi htkbook.pdf  (but linked references will fail)

htkbook.ps: htkbook.dvi
	dvips -t a4 -K -o htkbook.ps htkbook.dvi

htkbook.dvi: $(maintex) $(reftex) $(figures)
	latex htkbook
	latex htkbook
	makeindex htkbook.idx
	latex htkbook

html: htkbook.dvi
	-mkdir -p htkbook
	latex2html -t "HTKBook for HTK3" -address "<A HREF="http://htk.eng.cam.ac.uk/docs/docs.shtml" TARGET="_top">Back to HTK site</A><BR>See front page for HTK Authors" -white -transparent -html_version 4.0,frame  -dir htkbook htkbook.tex

all: htkbook.ps htkbook.pdf html

.PHONY: clean
clean:
	-rm -f *.aux *.log *.toc htkbook.i* *.dvi *.ps *.pdf \
	       *.ps.gz *.pdf.gz \
	       HTKRef/*.aux htkbook-pdf.*
	-rm -rf htkbook/

