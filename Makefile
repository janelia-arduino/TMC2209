.PHONY: clean
clean:
	rm -rf .pio

.PHONY: teensy-firmware
teensy-firmware: clean
	pio run -e teensy40

.PHONY: teensy-upload
teensy-upload: clean
	pio run -e teensy40 --target upload --upload-port /dev/ttyACM0

.PHONY: mega-firmware
mega-firmware: clean
	pio run -e mega

.PHONY: mega-upload
mega-upload: clean
	pio run -e mega --target upload --upload-port /dev/ttyACM0

.PHONY: uno-firmware
uno-firmware: clean
	pio run -e uno

.PHONY: uno-upload
uno-upload: clean
	pio run -e uno --target upload --upload-port /dev/ttyACM0

.PHONY: pico-firmware
pico-firmware: clean
	pio run -e pico

.PHONY: pico-upload
pico-upload: clean
	pio run -e pico --target upload

.PHONY: monitor
monitor:
	pio device monitor --echo --eol=LF
