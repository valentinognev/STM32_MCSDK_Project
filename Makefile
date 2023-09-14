.PHONY: all build build-container cmake format format-linux flash-stlink flash-jlink format-container shell image build-container clean clean-image clean-all
############################### Native Makefile ###############################

PROJECT_NAME ?= STM32_MCSDK_Project
BUILD_DIR ?= build
FIRMWARE := $(BUILD_DIR)/$(PROJECT_NAME).bin
BUILD_TYPE ?= Debug
PLATFORM = $(if $(OS),$(OS),$(shell uname -s))

ifeq ($(PLATFORM),Windows_NT)
    BUILD_SYSTEM ?= MinGW Makefiles
else
    ifeq ($(PLATFORM),Linux)
        BUILD_SYSTEM ?= Unix Makefiles
    else
        @echo "Unsuported platform"
        exit 1
    endif
endif

all: build

build: cmake
	$(MAKE) -C $(BUILD_DIR) --no-print-directory

cmake: $(BUILD_DIR)/Makefile

$(BUILD_DIR)/Makefile: CMakeLists.txt
	cmake \
		-G "$(BUILD_SYSTEM)" \
		-B$(BUILD_DIR) \
		-DPROJECT_NAME=$(PROJECT_NAME) \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DDUMP_ASM=OFF

# Formats all user modified source files (add ones that are missing)
SRCS := $(shell find Project -name '*.[ch]' -or -name '*.[ch]pp') Core/Src/main.c
format: $(addsuffix .format,$(SRCS))
%.format: %
	clang-format -i $<

# Formats all CubeMX generated sources to unix style - removes \r from line endings
# Add any new directories, like Middlewares and hidden files
HIDDEN_FILES := .mxproject .project .cproject
FOUND_HIDDEN_FILES := $(shell for f in $(HIDDEN_FILES);do if [[ -e $$f ]]; then echo $$f;fi; done)
FORMAT_LINUX := $(shell find Core Drivers -name '*' -type f; find . -name '*.ioc') $(FOUND_HIDDEN_FILES)

format-linux: $(addsuffix .format-linux,$(FORMAT_LINUX))
%.format-linux: %
	$(if $(filter $(PLATFORM),Linux),dos2unix -q $<,)

# Device specific!
DEVICE ?= STM32G431CBUx

flash-st: build
	st-flash --reset write $(FIRMWARE) 0x08000000

flash-ocd: build
	openocd 			\
	     -c "adapter serial 066CFF313050353043214925"   \
	     -f ./openocd.cfg -c "program $(BUILD_DIR)/$(PROJECT_NAME).elf verify reset exit"
# 066FFF313050353043205739 066CFF313050353043214925 
# 3000590010000059334A4D4E 48FF6D066567495726091187 1D17040029135147324D4E00		 
$(BUILD_DIR)/jlink-script:
	touch $@
	@echo device $(DEVICE) > $@
	@echo si 1 >> $@
	@echo speed 4000 >> $@
	@echo loadfile $(FIRMWARE),0x08000000 >> $@
	@echo -e "r\ng\nqc" >> $@

flash-jlink: build | $(BUILD_DIR)/jlink-script
	JLinkExe -commanderScript $(BUILD_DIR)/jlink-script

clean:
	rm -rf $(BUILD_DIR)

################################## Container ##################################

UID ?= $(shell id -u)
GID ?= $(shell id -g)
USER ?= $(shell id -un)
GROUP ?= $(if $(filter $(PLATFORM), Windows_NT),$(shell id -un),$(shell id -gn))

ifeq ($(PLATFORM),Windows_NT)
    WIN_PREFIX = winpty
    WORKDIR_PATH = "//workdir"
    WORKDIR_VOLUME = "/$$(pwd -W):/workdir"
else
    WORKDIR_PATH = /workdir
    WORKDIR_VOLUME = "$$(pwd):/workdir"
endif

CONTAINER_TOOL ?= docker
CONTAINER_FILE := Dockerfile
IMAGE_NAME := fedora-arm-embedded-dev
CONTAINER_NAME := fedora-arm-embedded-dev

NEED_IMAGE = $(shell $(CONTAINER_TOOL) image inspect $(IMAGE_NAME) 2> /dev/null > /dev/null || echo image)
# usefull if you have a always running container in the background: NEED_CONTAINER = $(shell $(CONTAINER_TOOL) container inspect $(CONTAINER_NAME) 2> /dev/null > /dev/null || echo container)
PODMAN_ARG = $(if $(filter $(CONTAINER_TOOL), podman),--userns=keep-id,)
CONTAINER_RUN = $(WIN_PREFIX) $(CONTAINER_TOOL) run \
				--name $(CONTAINER_NAME) \
				--rm \
				-it \
				$(PODMAN_ARG) \
				-v $(WORKDIR_VOLUME) \
				-w $(WORKDIR_PATH) \
				--security-opt label=disable \
				--hostname $(CONTAINER_NAME) \
				$(IMAGE_NAME)

build-container: $(NEED_IMAGE)
	$(CONTAINER_RUN) bash -lc 'make -j$(shell nproc)'

format-container:
	$(CONTAINER_RUN) bash -lc 'make format -j$(shell nproc)'

format-linux-container:
	$(CONTAINER_RUN) bash -lc 'make format-linux'

shell:
	$(CONTAINER_RUN) bash -l

image: $(CONTAINER_FILE)
	$(CONTAINER_TOOL) build \
		-t $(IMAGE_NAME) \
		-f=$(CONTAINER_FILE) \
		--build-arg UID=$(UID) \
		--build-arg GID=$(GID) \
		--build-arg USERNAME=$(USER) \
		--build-arg GROUPNAME=$(GROUP) \
		.

clean-image:
	$(CONTAINER_TOOL) container rm -f $(CONTAINER_NAME) 2> /dev/null > /dev/null || true
	$(CONTAINER_TOOL) image rmi -f $(IMAGE_NAME) 2> /dev/null > /dev/null || true

clean-all: clean clean-image
