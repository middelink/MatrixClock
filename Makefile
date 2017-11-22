all:	
	platformio run -t size

upload:
	platformio run -t upload --upload-port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A100JOEA-if00-port0

ota:
	PLATFORMIO_UPLOAD_FLAGS="-P8080, --auth=NoWay" platformio run -t upload --upload-port esp8266-da4035.local # - bureau

ota2:
	PLATFORMIO_UPLOAD_FLAGS="-P8080, --auth=NoWay" platformio run -t upload --upload-port esp8266-d5e24f.local # - kantoor

ota3:
	PLATFORMIO_UPLOAD_FLAGS="-P8080, --auth=NoWay" platformio run -t upload --upload-port esp8266-11c872.local # - woonkamer

clean:
	platformio run -t clean
	rm -rf .pioenvs .piolibdeps
