# ERA-7B1-Battery-Reader
Sony AIBO Battery Refill Tool

Sony stopped selling the batteries for their Aibo dogs. Sad to hear that this state of the art Aibo dog will be completely gone in the near future. The idea is refilling the old Aibo batteries. Here is step one to to keep Aibo alive. This tool will help to read the battery information. 


Battery connections (ERA-Battery):
If you number the pins from the one nearest the edge is 1 and the on furthest as 6 then:

1= Plus (B+)
2= SMBus clock
3= SMBus Data
4= Activate (B+) Has to be connected to pin 6(B-) to activate Pin 1 (B+)
5= Reserve
6= Minus (B-)

Arduino connections (Nano V3.1):

Connect pin 2 (ERA-Battery) SMBus-Clock to analog pin 5 on the Arduino, and pin 3 (ERA-Battery) SMBus-Data to analog pin 4. 
On both the SCL and SDA lines, add 4.7K ohm pull up resistors, connecting both lines to +3,3 V.

I hope you've been honest with yourself about your electronic skills. There is no warranty here. Use this tool at your own risk.

Special thanks to Chris69, he brought me to look deeper into the battery programming :-)
