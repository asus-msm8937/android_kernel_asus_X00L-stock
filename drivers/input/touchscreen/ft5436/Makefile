#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.
#obj-y += fts_ts.o 
#fts_ts-y += focaltech_core.o focaltech_ctl.o focaltech_ex_fun.o focaltech_gesture.o focaltech_flash.o 

obj-$(CONFIG_TOUCHSCREEN_QCOM_FT5436) += fts_ts.o self-test/

fts_ts-$(CONFIG_K89218_FEATURES) += focaltech_core.o focaltech_ctl.o focaltech_ex_fun.o focaltech_gesture.o focaltech_flash.o Asus_501_5436_V06_D01_20160721_app.i
fts_ts-$(CONFIG_K89200_FEATURES) += focaltech_core.o focaltech_ctl.o focaltech_ex_fun.o focaltech_gesture.o focaltech_flash.o ZC553_5436_V06_D01_20160721_app.i
fts_ts-$(CONFIG_K89219_FEATURES) += focaltech_core.o focaltech_ctl.o focaltech_ex_fun.o focaltech_gesture.o focaltech_flash.o

