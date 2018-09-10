ifeq ($(CONFIG_OF_OVERLAY),y)

dtbo-$(CONFIG_ARCH_S5P6818) += \
	s5p6818-artik710-raptor-spi-slave.dtbo

targets	+= dtbs dtbs_install
targets	+= $(dtb-y)

endif

always		:= $(dtbo-y)
clean-files	:= *.dtbo
