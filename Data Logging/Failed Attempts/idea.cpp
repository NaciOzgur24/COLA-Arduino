

// Send the command to the USB key. Wait a few ms for the
// USB interface to respond.
usb.print(cmd);
delay(10);

// Write any response to the monitor
while (usb.available())
{

    // Don't allow the USB interface internal buffer to overflow.
    if (usb.available() > 8)
    {
        digitalWrite(usbcts, HIGH);
    }
    else
    {
        digitalWrite(usbcts, LOW);
    }

    // Read a character from the USB device
    ch = usb.read();

    // Echo character to the serial monitor
    Serial.write(ch);
}


