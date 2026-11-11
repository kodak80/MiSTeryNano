#
# Unit tests for the HFE floppy blocks (src/fdc_hfe) against the C++
# reference model in tools/hfev3. Needs Verilator >= 5.015.
#
#   make                         build ./tb_hfe
#   make test IMAGES="a.hfe ..." run every side of every image
#

HDL_DIR = ../../src/fdc_hfe
HDL     = hfe_tb_top.v $(HDL_DIR)/hfe_player.v $(HDL_DIR)/wd1772_dpll.v $(HDL_DIR)/mfm_amdetect.v

IMAGES ?=

FDC_HDL = fdc_tb_top.v $(HDL_DIR)/hfe_player.v $(HDL_DIR)/wd1772_dpll.v $(HDL_DIR)/mfm_amdetect.v \
          $(HDL_DIR)/wd1772_mfm.v

FLOPPY_HDL = floppy_tb_top.v $(HDL_DIR)/hfe_floppy.v $(HDL_DIR)/hfe_player.v $(HDL_DIR)/wd1772_dpll.v \
             $(HDL_DIR)/mfm_amdetect.v $(HDL_DIR)/wd1772_mfm.v

all: tb_hfe tb_fdc tb_floppy

tb_hfe: tb_hfe.cpp ref.h ../../tools/hfev3/hfe_model.cpp $(HDL) Makefile
	verilator -O3 -Wall -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME --no-timing --top-module hfe_tb_top \
	  -cc $(HDL) --exe tb_hfe.cpp -o ../tb_hfe -CFLAGS "-O2 -std=c++17 -Wno-unused-function"
	$(MAKE) -s -C obj_dir -f Vhfe_tb_top.mk

# the controller test builds in its own directory
tb_fdc: tb_fdc.cpp ref.h ../../tools/hfev3/hfe_model.cpp $(FDC_HDL) Makefile
	verilator -O3 -Wall -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY --no-timing \
	  --top-module fdc_tb_top --Mdir obj_fdc -cc $(FDC_HDL) --exe tb_fdc.cpp -o ../tb_fdc \
	  -CFLAGS "-O2 -std=c++17 -Wno-unused-function"
	$(MAKE) -s -C obj_fdc -f Vfdc_tb_top.mk

tb_floppy: tb_floppy.cpp ref.h ../../tools/hfev3/hfe_model.cpp $(FLOPPY_HDL) Makefile
	verilator -O3 -Wall -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY --no-timing \
	  --top-module floppy_tb_top --Mdir obj_floppy -cc $(FLOPPY_HDL) --exe tb_floppy.cpp -o ../tb_floppy \
	  -CFLAGS "-O2 -std=c++17 -Wno-unused-function"
	$(MAKE) -s -C obj_floppy -f Vfloppy_tb_top.mk

test: tb_hfe
	./tb_hfe -q $(IMAGES)

clean:
	rm -rf obj_dir obj_fdc obj_floppy tb_hfe tb_fdc tb_floppy

.PHONY: all test clean
