KDIR      ?= $(HOME)/OSS/common
WORKSPACE ?= $(HOME)/OSS
ARCH      := arm64

obj-m := overclock_mt6768.o

CLANG_PREBUILT_BIN := $(shell \
  grep -E "^CLANG_PREBUILT_BIN=" $(WORKSPACE)/common/build.config.common \
  2>/dev/null | cut -d= -f2 | head -1)

ifneq ($(CLANG_PREBUILT_BIN),)
  CLANG_PATH := $(WORKSPACE)/$(CLANG_PREBUILT_BIN)
else
  CLANG_PATH := $(shell ls -d $(WORKSPACE)/prebuilts/clang/host/linux-x86/clang-r*/bin \
    2>/dev/null | sort -V | tail -1)
endif

CC            := $(CLANG_PATH)/clang
CLANG_TRIPLE  := aarch64-linux-gnu-
CROSS_COMPILE := aarch64-linux-gnu-
LLVM          := 1

# Optional cap overrides, e.g. make OC_DEFS="-DPOSDIV_2_MAX_FREQ=1400000U -DMAX_OC_PERCENT_OVER_STOCK=25"
OC_DEFS       ?=

EXTRA_CFLAGS  := \
  $(OC_DEFS) \
  --target=aarch64-linux-gnu \
  -fno-sanitize=shadow-call-stack \
  -Wno-unused-variable \
  -Wno-unused-function \
  -Wno-int-to-pointer-cast \
  -Wno-pointer-to-int-cast \
  -Wno-unused-command-line-argument

all:
	@if [ -z "$(CLANG_PATH)" ]; then \
		echo "ERROR: Clang not found under GKI prebuilts — check WORKSPACE ($(WORKSPACE)) and repo sync."; \
		echo "  Run 'make check-clang WORKSPACE=...' to diagnose."; \
		exit 1; \
	fi
	@echo "  [overclock_mt6768] Using Clang: $(CLANG_PATH)/clang"
	@$(CLANG_PATH)/clang --version 2>/dev/null | head -1
	PATH="$(CLANG_PATH):$$PATH" $(MAKE) -C $(KDIR) M=$(CURDIR) \
	  ARCH=$(ARCH) \
	  CC=$(CC) \
	  CLANG_TRIPLE=$(CLANG_TRIPLE) \
	  CROSS_COMPILE=$(CROSS_COMPILE) \
	  LLVM=1 \
	  LLVM_IAS=1 \
	  EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	  modules
	@if [ -x "$(CLANG_PATH)/llvm-strip" ]; then \
		$(CLANG_PATH)/llvm-strip --strip-debug overclock_mt6768.ko; \
		echo "  [overclock_mt6768] Stripped debug info:"; \
		ls -lh overclock_mt6768.ko; \
	else \
		echo "WARNING: llvm-strip not found at $(CLANG_PATH) — shipping unstripped .ko."; \
	fi

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

install:
	adb push overclock_mt6768.ko /data/local/tmp/
	adb shell su -c "insmod /data/local/tmp/overclock_mt6768.ko"
	@sleep 1
	adb shell su -c "dmesg | grep oc_mt6768 | tail -20"

uninstall:
	adb shell su -c "rmmod overclock_mt6768 && echo OK"

check-clang:
	@echo "WORKSPACE    : $(WORKSPACE)"
	@if [ -z "$(CLANG_PATH)" ]; then \
		echo "CLANG_PATH   : NOT FOUND — check WORKSPACE and repo sync"; \
		echo "Clang version: n/a"; \
	else \
		echo "CLANG_PATH   : $(CLANG_PATH)"; \
		echo "Clang version: $$($(CLANG_PATH)/clang --version 2>/dev/null | head -1)"; \
	fi
	@echo "KDIR         : $(KDIR)"
	@echo ""
	@CFI_LINE=$$(grep -E "^CONFIG_CFI_CLANG=" $(KDIR)/.config 2>/dev/null); \
	if [ -f "$(KDIR)/.config" ]; then \
		if [ "$$CFI_LINE" = "CONFIG_CFI_CLANG=y" ]; then \
			echo "CONFIG_CFI_CLANG: y (kbuild will inject -fsanitize=kcfi automatically)"; \
		else \
			echo "WARNING: CONFIG_CFI_CLANG is not 'y' in \$$(KDIR)/.config."; \
			echo "  If the kernel you're flashing IS CFI-enabled but this .config"; \
			echo "  disagrees with it, insmod can CFI-fault and reboot with no panic log."; \
		fi; \
	else \
		echo "WARNING: $(KDIR)/.config not found — cannot verify CONFIG_CFI_CLANG."; \
		echo "  Make sure KDIR points at a completed, configured kernel build tree."; \
	fi
