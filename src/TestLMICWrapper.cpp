/*
 * TestLMICWrapper
 *
 * - LMIC callbacks with CallbackRegister
 * - standby with EnergyController
 * - timer interrupt with ISRTimer
 */

#include <Arduino.h>
#include <arduino_lmic_hal_boards.h>
#include <Wire.h>

// leuville-arduino-easy-lmic
#include <LMICWrapper.h>
// leuville-arduino-utilities
#include <misc-util.h>
#include <ISRWrapper.h>
#include <ISRTimer.h>
#include <energy.h>
#include <StatusLed.h>
#include <CallbackRegister.h>

namespace lstl = leuville::simple_template_library;
namespace lora = leuville::lora;

using namespace lstl;
using namespace lora;

// LoRaWAN configuration: see OTAAId in LMICWrapper.h
#include <lora-common-defs.h>

USBPrinter<Serial_> console(STDOUT);

/* ---------------------------------------------------------------------------------------
 * Application classes
 * ---------------------------------------------------------------------------------------
 */

constexpr uint32_t _24h = 24 * 60 * 60;
constexpr uint32_t _7d = 7 * _24h;

/*
 * LoraWan endnode with:
 * - a timer to trigger a PING message on a regular basis
 * - a callback set on button connected to A0 pin, which triggers a BUTTON message
 * - standby mode capacity
 */
using Base = LMICWrapper;
using Button1 = ISRWrapper<DEVICE_BUTTON1_ISR>;

class EndNode : public Base, 
				private ISRTimer, 
				private Button1, 
				private EnergyController 
{
	/*
	 * Jobs for LMIC event callbacks
	 */
	enum JOB { BUTTON, TIMEOUT, JOIN, TXCOMPLETE, _COUNT };
	CallbackRegister<EndNode,JOB::_COUNT> _callbacks;

	int _count = 0;	// message counter

public:

	EndNode(RTCZero & rtc, const lmic_pinmap &pinmap): EndNode(rtc, &pinmap) {}

	EndNode(RTCZero & rtc, const lmic_pinmap *pinmap = Arduino_LMIC::GetPinmap_ThisBoard()): 
		Base(pinmap),
		ISRTimer(rtc, DEVICE_MEASURE_DELAY, true),
		Button1(INPUT_PULLUP, LOW),
		EnergyController(rtc)
	{
		_callbacks.set(BUTTON, 		this, &EndNode::buttonJob);
		_callbacks.set(TIMEOUT, 	this, &EndNode::timeoutJob);
		_callbacks.set(JOIN, 		this, &EndNode::joinJob);
		_callbacks.set(TXCOMPLETE, 	this, &EndNode::txCompleteJob);
	}

	/*
	 * delegates begin() to each sub-component and send a first message
	 */
	virtual void begin(const OTAAId& id, u4_t network, bool adr = true) override {
		Wire.begin();
		Base::begin(id, network, adr);
		Button1::begin();
		EnergyController::begin();
		ISRTimer::begin(true);

		Button1::enable();
		ISRTimer::enable();
		EnergyController::enable();
		startJoining();
	}

	/*
	 * Configure LoRaWan network (channels, power, adr, ...)
	 */
	virtual void initLMIC(u4_t network = 0, bool adr = true) override {
		configureNetwork(static_cast<Network>(network), adr);
	}

	/*
	 * Button ISR
	 * job done by sending a LMIC callback
	 * see completeJob()
	 */
	virtual void ISR_callback(uint8_t pin) override {
		setCallback(_callbacks[BUTTON]);
	}

	/*
	 * Timer ISR
	 * job done by sending a LMIC callback
	 * see completeJob()
	 */
	virtual void ISR_timeout() override {
		setCallback(_callbacks[TIMEOUT]);
	}

	/*
	 * Job done on join/unjoin
	 * see completeJob()
	 */
	virtual void joined(bool ok) override {
		if (ok) {
			setCallback(_callbacks[JOIN]);
		}
	}

	/*
	 * Updates the system time
	 */
	virtual void updateSystemTime(uint32_t newTime) override {
		ISRTimer::setEpoch(newTime);
		RTCZero & rtc = ISRTimer::getRTC();
		console.print("heure network=",	rtc.getHours()); 
		console.print(":", rtc.getMinutes());
		console.println(":", rtc.getSeconds());
	}

	/*
	 * Updates the timer delay
	 */
	virtual void downlinkReceived(const DownstreamMessage & message)  {
		if (message._len > 0) {
			uint32_t timerDelay = (uint32_t)strtoul((char*)message._buf, nullptr, 10);
			console.println("timerDelay: ", timerDelay);
			ISRTimer::setTimeout(timerDelay);
		}
	}

	/*
	 * Handle LMIC callbacks
	 */
	virtual void completeJob(osjob_t* job) override {
		_callbacks.execute(job);
	}

	virtual void buttonJob() {
		send("CLICK", true);
	}

	virtual void timeoutJob() {
		const char* format = "TIMEOUT %d";
		char msg[80];
		sprintf(msg, format, _count++);
		send(msg, false);
	}

	virtual void joinJob() {
		LoRaWanSessionKeys keys = getSessionKeys();
		// https://www.thethingsnetwork.org/docs/lorawan/prefix-assignments.html
		console.println("netId: ", keys._netId, HEX);
		console.println("devAddr: ", keys._devAddr, HEX);
		console.print("nwkSKey: "); console.printHex(keys._nwkSKey, arrayCapacity(keys._nwkSKey));
		console.print("appSKey: "); console.printHex(keys._appSKey, arrayCapacity(keys._appSKey));
		postJoinSetup(keys._netId);
	}

	virtual void txCompleteJob() {
		console.println("FIFO size: ", _messages.size());
	}

	/*
	 * Build and send Uplink message
	 */
	void send(const char* message, bool ack = false) {
		UpstreamMessage payload((uint8_t*)message, strlen(message)+1, ack);
		console.print("send ", message);
		Base::send(payload);
		console.println(", FIFO size: ", _messages.size());
	}

	/*
	 * LMIC callback called on TX_COMPLETE event
	 */
	virtual bool isTxCompleted(const UpstreamMessage & message, bool ack) override {
		setCallback(_callbacks[TXCOMPLETE]);
		console.print("isTxCompleted ", (char*)message._buf); 
		console.println(" / ack: ", ack);
		return Base::isTxCompleted(message, ack);
	}

	/*
	 * Activates standby mode
	 */
	void standby() {
		EnergyController::standby();
	}

};

/* ---------------------------------------------------------------------------------------
 * GLOBAL OBJECTS
 * ---------------------------------------------------------------------------------------
 */

RTCZero rtc;

BlinkingLed statusLed(LED_BUILTIN, 500);

#if defined(LMIC_PINS)
EndNode 	endnode(rtc, LMIC_PINS); 
#else
EndNode 	endnode(rtc);
#endif

/* ---------------------------------------------------------------------------------------
 * ARDUINO FUNCTIONS
 * ---------------------------------------------------------------------------------------
 */
void setup()
{
	console.begin(115200);

	statusLed.begin();
	statusLed.on();

	while (!console.available()) { delay(10); }

	endnode.begin(id[DEVICE_CONFIG], DEVICE_NETWORK, ADR::ON);

	statusLed.off();
}

void loop()
{
	endnode.runLoopOnce();
	if (endnode.isReadyForStandby()) {
		statusLed.off();
		#ifndef DEVICE_DEBUG_MODE
		endnode.standby(); 
		#endif
	}
	else {
		statusLed.blink();
	}
}
