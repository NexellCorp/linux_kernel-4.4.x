ifeq ($(CONFIG_OF_OVERLAY),y)

dtbo-$(CONFIG_ARCH_S5P4418) += \
	s5p4418-artik533-compy-i2c1.dtbo	\
	s5p4418-artik533-compy-serial4.dtbo	\
	s5p4418-artik533-compy-spi2.dtbo	\
	s5p4418-artik533-compy-sense.dtbo

targets	+= dtbs dtbs_install
targets	+= $(dtb-y)

endif

always		:= $(dtbo-y)
clean-files	:= *.dtbo
