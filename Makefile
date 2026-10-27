# bm-modbus: Modbus RTU on top of the BareMetal SDK (sdk/ is a git submodule)
#
#   make TARGET=qemu                 build
#   make TARGET=qemu run             build and run under QEMU
#   make TARGET=riser TFTPROOT=/srv/tftp netboot   copy the image as boot.img
#   make test                        host tests (gcc + sanitizers, no SDK needed to run)
#
#   make TARGET=qemu ROLE=master DEMO_FLAGS="-DMODBUS_UART_BASE=0x..."   master instead of slave
# Demo options go in DEMO_FLAGS, e.g.  make DEMO_FLAGS="-DMODBUS_DEMO_RS485 -DMODBUS_DEMO_DE_LEAD_US=20"

TARGET ?= qemu

SDK_ROOT  := $(CURDIR)/sdk
SDK_SUB   := $(SDK_ROOT)/sdk

include $(SDK_SUB)/build.mk
# build.mk works out its paths from where the file sits, lazily, so once other
# makefiles are included it would point at the wrong place. Pin them.
SDK_DIR      := $(SDK_SUB)
PROJECT_ROOT := $(SDK_ROOT)

SDK_BUILD := $(SDK_ROOT)/build
OUT       := $(CURDIR)/build
ROLE      ?= slave
ifeq ($(ROLE),master)
ROLE_FLAGS := -DMODBUS_ROLE_MASTER
else ifeq ($(ROLE),slave)
ROLE_FLAGS :=
else
$(error ROLE must be slave or master)
endif
NAME      := modbus_$(ROLE)

SRCS := $(wildcard src/*.c) $(wildcard demo/*.c)
OBJS := $(patsubst %.c,$(OUT)/obj/%.$(TARGET).o,$(SRCS))
LIB  := $(SDK_BUILD)/libplatform_$(TARGET).a
LD   := $(SDK_BUILD)/ldscripts/bmmap.$(TARGET).ld

DEMO_FLAGS ?=
APP_CFLAGS = $(CFLAGS) -DDEBUG -fno-reorder-blocks-and-partition $(ROLE_FLAGS) $(DEMO_FLAGS) -I include -I $(SDK_TARGETS_DIR)/$(TARGET)

.PHONY: all sdk-patch sdk-lib run netboot clean test FORCE
.DEFAULT_GOAL := all

all: $(OUT)/$(NAME).$(TARGET).bin

sdk-patch:
	@bash scripts/apply-sdk-patches.sh

# Builds only this target's library and linker script inside the submodule.
sdk-lib: sdk-patch
	$(MAKE) -C $(SDK_SUB) -f sdk.mk $(LIB) $(LD)

# Rebuild when ROLE or DEMO_FLAGS change (make would otherwise keep the old objects)
FLAGS_STAMP := $(OUT)/flags.$(TARGET).txt
$(FLAGS_STAMP): FORCE
	@mkdir -p $(OUT)
	@echo '$(ROLE) $(DEMO_FLAGS)' | cmp -s - $@ || echo '$(ROLE) $(DEMO_FLAGS)' > $@
FORCE:
$(OBJS): $(FLAGS_STAMP)

# -fno-lto: the application objects are compiled normally. With LTO, GCC 16 / binutils 2.46 fail on the
# slave engine ("Error: .size expression for finish_frame ... does not evaluate to a constant").
# The SDK library is still built and linked with LTO.
$(OUT)/obj/%.$(TARGET).o: %.c | sdk-lib
	@mkdir -p $(dir $@)
	$(MSG) "  [CC]   $@"
	$(Q)$(CC) $(APP_CFLAGS) -fno-lto -c $< -o $@

$(OUT)/$(NAME).$(TARGET).elf: $(OBJS) sdk-lib
	$(MSG) "  [LD]   $@"
	$(Q)$(CC) $(APP_CFLAGS) $(OBJS) $(call PLATFORM_LIB,$(TARGET)) -o $@ $(LOPTS) \
		-Wl,-u,memset -Wl,-u,memcpy -Wl,-u,memmove -Wl,-u,memcmp -T $(LD)

$(OUT)/$(NAME).$(TARGET).bin: $(OUT)/$(NAME).$(TARGET).elf
	$(MSG) "  [BIN]  $@"
	$(Q)$(OBJCOPY) $(CPOPS) $< $@

run: $(OUT)/$(NAME).$(TARGET).bin
	ORIGINAL_PWD=$(CURDIR) bash $(SDK_TARGETS_DIR)/$(TARGET)/run.sh $<

netboot: $(OUT)/$(NAME).$(TARGET).bin
ifndef TFTPROOT
	$(error TFTPROOT not set, e.g. make TARGET=riser TFTPROOT=/srv/tftp netboot)
endif
	cp $< $(TFTPROOT)/boot.img
	@echo "Copied to $(TFTPROOT)/boot.img"

test:
	@bash scripts/host-test.sh

clean:
	rm -rf $(OUT)