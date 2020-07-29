.PHONY: namote, nl073

namote: mbed-os
	pipenv run mbed compile -m MOTE_L152RC -t GCC_ARM --profile ./mbed-os/tools/profiles/release.json

nl073: mbed-os
	pipenv run mbed compile -m NUCLEO_L073RZ -t GCC_ARM --profile ./mbed-os/tools/profiles/release.json

mbed-os:
	pipenv install
	pipenv run mbed config root .
	pipenv run mbed deploy
	
