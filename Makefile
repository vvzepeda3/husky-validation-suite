# Unit tests and linters run with plain Python (no ROS install needed).
# build/test use colcon and need a sourced ROS 2 Jazzy environment, e.g. inside `make docker-shell`.

PYTHON ?= python3
ROS_IMAGE ?= osrf/ros:jazzy-desktop
PKG_SRC := src

.PHONY: help unit lint format build test clean docker-shell

help:
	@echo "make unit          Run unit tests (pytest, no ROS required)"
	@echo "make lint          ruff check + black --check"
	@echo "make format        Apply black and ruff --fix"
	@echo "make build         colcon build (needs ROS 2 Jazzy sourced)"
	@echo "make test          colcon test  (needs ROS 2 Jazzy sourced)"
	@echo "make docker-shell  Shell in a ROS 2 Jazzy container with this repo at /ws"
	@echo "make clean         Remove colcon build/install/log"

# pytest exits 5 when no tests are collected; that's fine until the first test lands.
unit:
	@$(PYTHON) -m pytest -m "not integration" $(PKG_SRC); status=$$?; \
	if [ $$status -eq 5 ]; then echo "No unit tests yet."; exit 0; fi; exit $$status

lint:
	$(PYTHON) -m ruff check $(PKG_SRC)
	$(PYTHON) -m black --check $(PKG_SRC)

format:
	$(PYTHON) -m black $(PKG_SRC)
	$(PYTHON) -m ruff check --fix $(PKG_SRC)

build:
	colcon build --symlink-install

test:
	colcon test && colcon test-result --verbose

clean:
	rm -rf build install log

# Override the image if you use your own: make docker-shell ROS_IMAGE=my/jazzy-image
docker-shell:
	docker run -it --rm -v "$(CURDIR)":/ws -w /ws $(ROS_IMAGE) bash
