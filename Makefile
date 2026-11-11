# HX-1 pipeline. `make all` regenerates every artefact from config/robot.yaml and runs the QA tree.
PY ?= python3
FW := firmware

.PHONY: all loop gen cad render videos fw-test fw-esp32 sil docs qa qa-fast test clean

all: loop gen render fw-test sil videos docs qa-fast test

loop:            ## recursive design loop: sim -> sizing -> CAD -> mass properties (until converged)
	$(PY) -m hx loop
gen:             ## firmware headers, CAN protocol, pin map, netlists, power/harness reports
	$(PY) -m hx gen
cad:             ## STLs, plates, print plan, BOM, mass properties
	$(PY) -m hx cad
render:          ## assembly renders + parts sheet
	$(PY) -m hx render
videos:          ## MuJoCo task videos with CAD meshes -> docs/video (needs libegl1/libosmesa6 + ffmpeg)
	MUJOCO_GL=egl $(PY) -m hx video
fw-test:         ## host build (-Werror) + unit tests of the firmware core
	cmake -S $(FW) -B $(FW)/build -DCMAKE_BUILD_TYPE=Release
	cmake --build $(FW)/build -j4
	$(FW)/build/hx_tests
fw-esp32:        ## cross-compile all node firmwares (needs PlatformIO + network)
	cd $(FW)/platform/esp32s3 && pio run
sil:             ## hydraulic software-in-the-loop with the real firmware core
	$(PY) -m hx sil
docs:            ## regenerate docs/00_design_summary.md
	$(PY) -m hx docs
qa:              ## full recursive QA tree (re-runs SIL)
	$(PY) -m hx qa
qa-fast:         ## QA reusing cached SIL results when robot.yaml is unchanged
	$(PY) -m hx qa --fast
test:            ## python unit tests
	$(PY) -m pytest -q tests
clean:
	rm -rf $(FW)/build build .pytest_cache
