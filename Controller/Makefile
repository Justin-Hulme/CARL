# ------------------------------------------------------------
#  Keil µVision Build Makefile (Wine on Linux)
# ------------------------------------------------------------

WINEPREFIX := $(HOME)/wine-keil64
WINE := WINEPREFIX=$(WINEPREFIX) wine
UV4 := C:\\\\users\\\\hephaestus\\\\AppData\\\\Local\\\\Keil_v5\\\\UV4\\\\UV4.exe

# Automatically find the first .uvprojx file in the current directory if not specified
PROJECT_FILE := $(or $(PROJECT),$(firstword $(wildcard *.uvprojx)))

ifeq ($(PROJECT_FILE),)
$(error No .uvprojx project file found in the current directory. Use 'make PROJECT=yourproject.uvprojx')
endif

# Convert the project path to Windows format for Wine
PROJECT_WIN := $(shell winepath -w "$(abspath $(PROJECT_FILE))" | sed 's#\\\\#\\\\\\\\#g')

# Default target name (can be overridden: make TARGET="Target 2")
TARGET ?= Target 1

# Output directory for build artifacts
OUT_DIR := build

# ------------------------------------------------------------
# Targets
# ------------------------------------------------------------

all: build copy_outputs

# --- Build the project ---
build:
	@echo "────────────────────────────────────────────"
	@echo "Building: $(PROJECT_FILE)"
	@echo "Target:   $(TARGET)"
	@echo "Project:  $(PROJECT_WIN)"
	@echo "────────────────────────────────────────────"
	@echo
	@echo "Running command:"
	@echo "WINEDEBUG=-all WINEPREFIX=$(WINEPREFIX) wine \"$(UV4)\" -j0 -r \"$(PROJECT_WIN)\" -t \"$(TARGET)\""
	@echo "────────────────────────────────────────────"
	WINEDEBUG=-all WINEPREFIX=$(WINEPREFIX) wine "$(UV4)" -j0 -b "$(PROJECT_WIN)" -t "$(TARGET)" || (echo "❌ Build failed with exit code $$?"; exit 1)
	@echo
	@echo "✅ Build complete."
	@echo "────────────────────────────────────────────"

# --- Clean the project ---
clean:
	@echo "Cleaning $(PROJECT_FILE) for target $(TARGET)..."
# Uncomment this line to use Keil's built-in clean:
#	@WINEDEBUG=-all $(WINE) "$(UV4)" -j0 -c "$(PROJECT_WIN)" -t "$(TARGET)"
	@echo "Removing generated files and folders..."
	@rm -f *.o *.axf *.hex *.map *.lnp *.d *.htm *.build_log.htm
	@rm -rf Objects Listings $(OUT_DIR)
	@echo "Clean complete."

# --- Copy build artifacts (.hex, .axf) to ./build ---
copy_outputs:
	@mkdir -p $(OUT_DIR)
	@find . -maxdepth 3 -type f \( -iname "*.hex" -o -iname "*.axf" \) ! -path "./$(OUT_DIR)/*" -exec cp -u {} $(OUT_DIR)/ \; || true
	@echo "Copied build outputs (if any) to $(OUT_DIR)/"
	@echo

# --- Launch µVision GUI manually ---
gui:
	@echo "Launching µVision IDE..."
	@WINEDEBUG=-all WINEPREFIX=$(WINEPREFIX) wine "$(UV4)"

# --- Open the project directly in µVision ---
open:
	@echo "Opening $(PROJECT_FILE) in µVision..."
	@WINEDEBUG=-all WINEPREFIX=$(WINEPREFIX) wine "$(UV4)" "$(PROJECT_WIN)"

# --- Open the Keil build log in default browser ---
log:
	@echo "Opening Keil build log..."
	@xdg-open *.build_log.htm >/dev/null 2>&1 || echo "No build log found."

.PHONY: all build clean copy_outputs gui open log
