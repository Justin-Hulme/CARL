WINEPREFIX := $(HOME)/wine-keil64
WINE := WINEPREFIX=$(WINEPREFIX) wine
UV4 := C:\\users\\hephaestus\\AppData\\Local\\Keil_v5\\UV4\\UV4.exe

PROJECT_FILE := $(or $(PROJECT),$(firstword $(wildcard *.uvprojx)))

ifeq ($(PROJECT_FILE),)
$(error No .uvprojx project file found in the current directory. Use 'make PROJECT=yourproject.uvprojx')
endif

CURDIR_WIN := $(shell winepath -w "$(CURDIR)" | sed 's#\\\\#\\\\\\\\#g')
PROJECT := $(CURDIR_WIN)\\$(PROJECT_FILE)

TARGET ?= Target 1

OUT_DIR := Build

all: build copy_outputs

build:
	@echo "Building $(PROJECT_FILE) (Target: $(TARGET))..."
	@echo "Running command:"
	@echo "WINEDEBUG=-all $(WINE) \"$(UV4)\" -j0 -b \"$(PROJECT)\" -t \"$(TARGET)\""
	@WINEDEBUG=-all $(WINE) "$(UV4)" -j0 -b "$(PROJECT)" -t "$(TARGET)" || (echo "❌ Build failed with exit code $$?"; exit 1)

clean:
	@echo "Cleaning $(PROJECT_FILE) for target $(TARGET)..."
	@mkdir -p $(OUT_DIR)
	@find . -maxdepth 1 -type f \( -iname "*.hex" -o -iname "*.axf" \) -exec mv -u {} $(OUT_DIR)/ \; || true
	@rm -f *.o *.map *.lnp *.d *.htm *.build_log.htm
	@rm -rf Objects Listings
	@echo "Moved .hex and .axf files to $(OUT_DIR) and cleaned other build artifacts."

copy_outputs:
	@mkdir -p $(OUT_DIR)
	@find . -maxdepth 3 -type f \( -iname "*.hex" -o -iname "*.axf" \) ! -path "./$(OUT_DIR)/*" -exec cp -u {} $(OUT_DIR)/ \; || true
	@echo "Copied build outputs (if any) to $(OUT_DIR)/"
	@echo

gui:
	@echo "Launching µVision IDE..."
	@WINEDEBUG=-all $(WINE) "$(UV4)"

open:
	@echo "Opening $(PROJECT_FILE) in µVision..."
	@WINEDEBUG=-all $(WINE) "$(UV4)" "$(PROJECT)"

log:
	@echo "Opening Keil build log..."
	@xdg-open LCD.build_log.htm >/dev/null 2>&1 || echo "Failed to open build log. File may not exist."

.PHONY: all build clean copy_outputs gui open log
