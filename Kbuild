obj-m += espi_driver.o
espi_driver-y := espi_driver_core.o espi_fb.o espi_lpc8xx.o espi_ssd1322.o espi_ssd1305.o espi_buttons.o espi_ribbon_leds.o espi_leds.o espi_epc_ctrl.o espi_main_ctrl.o
ccflags-y += -DNL_VERSION=\"$(shell git -C $(M) rev-parse --short HEAD)\"
