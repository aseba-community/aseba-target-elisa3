avrdude -p m2560 -P COM69 -b 57600 -c stk500v2 -D -Uflash:w:elisa3-aseba.hex:i -v
pause