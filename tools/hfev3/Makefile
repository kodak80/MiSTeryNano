CXX ?= g++
CXXFLAGS ?= -O2 -Wall -Wextra

hfe_model: hfe_model.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -rf steem_check/build
	rm -f hfe_model

.PHONY: clean steem-check

# Cross-check against Steem SSE with HFEv3's decoder:
#   make steem-check STEEM=/path/to/Steem-SSE-with-HFEv3 IMAGES="a.hfe b.hfe"
STEEM ?= ../../../Steem-SSE-with-HFEv3
SB = steem_check/build
steem-check: hfe_model
	$(MAKE) -C $(STEEM)/tests/hfe B=$(abspath $(SB)) CXXFLAGS="-std=c++11 -O2 -w" $(abspath $(SB))/disk_hfe.cpp
	$(CXX) -std=c++11 -O2 -w -I$(SB) -o $(SB)/steem_dump steem_check/steem_dump.cpp steem_check/stubs.cpp $(SB)/disk_hfe.cpp
	@for f in $(IMAGES); do ./hfe_model -s "$$f" > $(SB)/model.txt; $(SB)/steem_dump "$$f" > $(SB)/steem.txt; \
	  printf "%-50s %5d fields, %d differ\n" "$$(basename $$f)" $$(wc -l < $(SB)/model.txt) $$(diff $(SB)/model.txt $(SB)/steem.txt | grep -c '^<'); done
