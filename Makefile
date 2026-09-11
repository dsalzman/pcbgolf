PYTHON ?= python3
PCB_TOOL := $(PYTHON) tools/pcbgolf.py
BUILD_DIR ?= .pcbgolf-build
PREPARED_BOARD := $(BUILD_DIR)/pcbgolf-prepared.kicad_pcb
PLACED_BOARD := $(BUILD_DIR)/pcbgolf-placed.kicad_pcb
AUTOROUTED_BOARD := $(BUILD_DIR)/pcbgolf-autorouted.kicad_pcb
ROUTED_BOARD := $(BUILD_DIR)/pcbgolf-routed.kicad_pcb

.PHONY: help env-check prepare place route finish test score pipeline unit-test

help:
	@echo "PCB Golf automation"
	@echo "  make env-check  Verify KiCad 10, Java 25, and Freerouting"
	@echo "  make prepare    Normalize rules, thickness, contacts, and outline"
	@echo "  make place      Apply the compact connectivity-aware floorplan"
	@echo "  make route      Autoroute the placed board"
	@echo "  make finish     Complete dense signals and ground-plane islands"
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
		--output $(AUTOROUTED_BOARD) \
		--work-dir $(BUILD_DIR)/route \
		--passes 20 \
		--fanout \
		--fanout-passes 1 \
		--via-cost 50 \
		--ripup-cost 100 \
		--neck-width-um 0

finish: route
	$(PCB_TOOL) finish \
		--input $(AUTOROUTED_BOARD) \
		--output $(ROUTED_BOARD)

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

pipeline: finish test score
