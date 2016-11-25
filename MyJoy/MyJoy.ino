/*
Blink: Turns on the built-in LED on for one second, then off for one second, repeatedly.
Arduino 1.6.0rc1
Sketch uses 11,900 bytes (11%) of program storage space. Maximum is 108,000 bytes.
Global variables use 2,592 bytes of dynamic memory.
Ported to Maple from the Arduino example 27 May 2011 By Marti Bolivar

Program size: 15 132 bytes (used 23% of a 65 536 byte maximum) (0.75 secs)
*/

//#include  "usbd_hid_core.h"
//#include  "usbd_usr.h"
//#include  "usbd_desc.h"


#define BAUD 115200


void setup() {
	//Serial1.begin(BAUD);

	//Serial1.println("Starting...");

	// Set up the built-in LED pin as an output:
	pinMode(PC13, OUTPUT);

	//Serial.begin();

	//while (!(Serial.isConnected() && (Serial.getDTR() || Serial.getRTS())))
	//{
	//	digitalWrite(PC13, !digitalRead(PC13));// Turn the LED from off to on, or on to off
	//	delay(100);         // fast blink
	//}

	//usb_init_usblib();

//	USBD_Init(&USB_OTG_dev,
//#ifdef USE_USB_OTG_HS 
//		USB_OTG_HS_CORE_ID,
//#else            
//		USB_OTG_FS_CORE_ID,
//#endif
//		&USR_desc,
//		&USBD_HID_cb,
//		&USR_cb);
//


	//Serial1.println("Started!");
}

void loop() {
	//Serial1.println("test");
	digitalWrite(PC13, !digitalRead(PC13));// Turn the LED from off to on, or on to off
	delay(1000);          // Wait for 1 second (1000 milliseconds)
}