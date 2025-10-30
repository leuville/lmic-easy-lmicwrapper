/*
 * TestLMICWrapper
 *
 * - LMIC callbacks with CallbackRegister
 * - battery power with EnergyController
 * - timer interrupt & standby with ISRTimer
 */

#include "EndNodeBase.h"
#include <StatusLed.h>

/* ---------------------------------------------------------------------------------------
 * Application classes
 * ---------------------------------------------------------------------------------------
 */

/*
 * LoraWan endnode with:
 * - a timer to trigger a PING message on a regular basis
 * - a callback set on button connected to A0 pin, which triggers a BUTTON message
 * - standby mode capacity
 */
using Base = EndNodeBase<LMICWrapper>;

class EndNode : public Base
{

	uint32_t _count = 0;
	
	const Range<u1_t> _rangeLora {MCMD_DEVS_BATT_MIN, MCMD_DEVS_BATT_MAX};

public:

	using Base::Base;

	/*
	 * delegates begin() to each sub-component and join
	 * 
	 * Do not forget Wire.begin() if I2C devices connected
	 */
	virtual void begin(const OTAAId& id, u4_t network, bool adr = true) override {
		setUnusedPins({A1, A2, A3, A4, A5});
		Base::begin(id, network, adr);
	}

	/*
	 * Updates the timer delay
	 */
	virtual void downlinkReceived(const DownstreamMessage & message) override  {
		if (message._len > 0) {
			uint32_t timerDelay = (uint32_t)strtoul((char*)message._buf, nullptr, 10);
			#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
			console.println("----------------------------------------------------------");
			console.println("timerDelay: ", timerDelay);
			console.println("----------------------------------------------------------");
			#endif
			ISRTimer::setTimeout(timerDelay);
		}
	}

	virtual void buttonJob() override {
		const char* format = "CLICK %d";
		char msg[80];
		sprintf(msg, format, Base::getBatteryPower(EnergyCtrl::_range100));
		send(msg, true);
	}

	virtual void timeoutJob() override {
		const char* format = "TIMEOUT %d";
		char msg[80];
		sprintf(msg, format, _count++);
		send(msg, false);
	}

	/*
	 * Build and send Uplink message
	 */
	void send(const char* message, bool ack = false) {
		auto batt = getBatteryPower(_rangeLora);	
		setBatteryLevel(batt);
		UpstreamMessage payload((uint8_t*)message, strlen(message)+1, ack);
		Base::send(payload);
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		console.println("----------------------------------------------------------");
		console.print("send ", message);
		console.println(", FIFO size: ", _messages.size());
		console.println("----------------------------------------------------------");
		#endif
	}

};

/* ---------------------------------------------------------------------------------------
 * GLOBAL OBJECTS
 * ---------------------------------------------------------------------------------------
 */

BlinkingLed statusLed(LED_BUILTIN, 500);

#if defined(LMIC_PINS)
EndNode 	endnode{LMIC_PINS}; 
#else
EndNode 	endnode{Arduino_LMIC::GetPinmap_ThisBoard()};
#endif

/* ---------------------------------------------------------------------------------------
 * ARDUINO FUNCTIONS
 * ---------------------------------------------------------------------------------------
 */
void setup()
{
	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
	console.begin(115200);
	#endif

	statusLed.begin();
	statusLed.on();

	endnode.begin(id[DEVICE_CONFIG], DEVICE_NETWORK, ADR::ON);

	delay(5000);
	statusLed.off();
}

void loop()
{
	endnode.runLoopOnce();
	if (endnode.isReadyForStandby()) {
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		statusLed.off();
		#else
		endnode.standby(); 
		#endif
	}
	else {
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		statusLed.blink();
		#endif
	}
}
