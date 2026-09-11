PYTHON ?= python3
PCB_TOOL := $(PYTHON) tools/pcbgolf.py
BUILD_DIR ?= .pcbgolf-build
PREPARED_BOARD := $(BUILD_DIR)/pcbgolf-prepared.kicad_pcb
PLACED_BOARD := $(BUILD_DIR)/pcbgolf-placed.kicad_pcb
ROUTED_BOARD := $(BUILD_DIR)/pcbgolf-routed.kicad_pcb

.PHONY: help env-check prepare place route test score pipeline unit-test

help:
	@echo "PCB Golf automation"
	@echo "  make env-check  Verify KiCad 10, Java 25, and Freerouting"
	@echo "  make prepare    Normalize rules, thickness, contacts, and outline"
	@echo "  make place      Apply the compact connectivity-aware floorplan"
	@echo "  make route      Autoroute the placed board"
	@echo "  make test       Run KiCad DRC and connectivity checks"
	@echo "  make score      Export the assembly and calculate challenge score"
	@echo "  make pipeline   Prepare, route, test, and score"

env-check:
	$(PCB_TOOL) env-check

prepare:
	$(PCB_TOOL) prepare --output $(PREPARED_BOARD)

place:
	$(PCB_TOOL) place --output $(PLACED_BOARD)

route: place
	$(PCB_TOOL) route \
		--input $(PLACED_BOARD) \
		--output $(ROUTED_BOARD) \
		--work-dir $(BUILD_DIR)/route \
		--passes 10 \
		--fanout \
		--fanout-passes 1 \
		--via-cost 20 \
		--ripup-cost 10

test:
	$(PCB_TOOL) test \
		--input $(ROUTED_BOARD) \
		--report $(BUILD_DIR)/drc.json

score:
	$(PCB_TOOL) score \
		--input $(ROUTED_BOARD) \
		--artifact-dir $(BUILD_DIR)/score

unit-test:
	$(PYTHON) -m unittest discover -s tests -v

pipeline: route test score
