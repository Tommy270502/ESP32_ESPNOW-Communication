# ESP32_ESPNOW-Communication
This Repository contains two Arduino sketches which make use of Espressifs ESPNOW communication protocol to send "1" and "0" from a Master ESP32 to a Slave ESP32 and vice versa.

Both the Master and the Slave can transmit and receive signals.

SKETCH PINOUT:
Pin[2] = INPUT -> reads signal to then send to the receiver.
Pin[23] = OUTPUT -> outputs signal received from transmitter.

RECEIVING:
When a "1" is received from the sender, then Pin [23] of the receiverESP will go "1".
When a "0" is received from the senderESP, then Pin [23} of the receiverESP will go "0".

SENDING:
When a "1" is read on Pin[2] the transmitter sends that signal to the receiver.
When a "0" is read on Pin[2] the transmitter sends that signal to the receiver.
