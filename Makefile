.PHONY: nmote, nucleo_l073

namote:
	mbed compile -m MOTE_L152RC -t GCC_ARM --profile ./mbed-os/tools/profiles/release.json

nucleo_l073:
	mbed compile -m NUCLEO_L073RZ -t GCC_ARM --profile ./mbed-os/tools/profiles/release.json