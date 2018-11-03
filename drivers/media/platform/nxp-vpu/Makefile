ifneq ($(CONFIG_VIDEO_NEXELL_CODEC),m)
obj-$(CONFIG_VIDEO_NEXELL_CODEC) := \
	nx_vpu_v4l2.o \
	nx_vpu_dec_v4l2.o \
	nx_vpu_enc_v4l2.o \
	nx_port_func.o \
	nx_vpu_api.o \
	nx_vpu_encoder.o \
	nx_vpu_decoder.o \
	nx_jpu_api.o \
	vpu_hw_interface.o \
	nx_vpu_gdi.o
else
MODULE := nx_vpu
obj-$(CONFIG_VIDEO_NEXELL_CODEC) := $(MODULE).o
$(MODULE)-objs += \
	nx_vpu_v4l2.o \
	nx_vpu_dec_v4l2.o \
	nx_vpu_enc_v4l2.o \
	nx_port_func.o \
	nx_vpu_api.o \
	nx_vpu_encoder.o \
	nx_vpu_decoder.o \
	nx_jpu_api.o \
	vpu_hw_interface.o \
	nx_vpu_gdi.o
endif
