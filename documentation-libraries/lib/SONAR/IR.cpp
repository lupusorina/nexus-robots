#include<IR.h>


IR::IR(unsigned char analog) {
	setPin(analog);
}
unsigned int IR::getDist() {
	if(getPin()==PIN_UNDEFINED) return 0xffff;
	_lastDist=6762/(analogRead(_analogPin)-9)-4;
	if(_lastDist<10 || _lastDist>80) _lastDist=0xffff;		// out of range
	return getLastDist();
}
unsigned int IR::getLastDist() const {
	return _lastDist;
}
unsigned char IR::setPin(unsigned char analog) {
	// assert();
	_analogPin=(analog<ANALOGMAX?analog:PIN_UNDEFINED);
	return getPin();
}
unsigned char IR::getPin() const {
	return _analogPin;
}



