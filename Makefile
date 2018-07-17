# Directories
BUILD_DIR := build
SRC_DIR := $(addprefix $(BUILD_DIR)/,scad)
MODEL_DIR := $(addprefix $(BUILD_DIR)/,model)

# match "module foobar() { // `make` me"
# Generates targets from base.scad
MODELS := $(shell sed '/^module [a-z0-9_-]*().*make..\?me.*$$/!d;s/module //;s/().*/.stl/' base.scad)
TARGETS := $(addprefix $(MODEL_DIR)/,$(MODELS))

# auto-generated .scad files with .deps make make re-build always. keeping the
# scad files solves this problem.
.SECONDARY: $(addprefix $(SRC_DIR)/,$(shell echo "${MODELS}" | sed 's/\.stl/.scad/g'))

# explicit wildcard expansion suppresses errors when no files are found
include $(wildcard *.deps)

all: ${TARGETS}

clean:
	rm -rfv ${BUILD_DIR}

$(SRC_DIR)/%.scad: base.scad
	echo -ne 'use <../../base.scad>\n$*();\n' > $@

$(MODEL_DIR)/%.stl: $(SRC_DIR)/%.scad
	openscad -m make -o $@ -d ${SRC_DIR}/$(shell basename $@).deps $<

$(TARGETS) : | $(BUILD_DIR) $(SRC_DIR) $(MODEL_DIR)

$(BUILD_DIR) $(SRC_DIR) $(MODEL_DIR):
	mkdir $@
