/*
 * TestLMICWrapper
 *
 * - LMIC callbacks with JobRegister
 * - battery power with EnergyController
 * - timer interrupt & standby with ISRTimer
 */

#include <Arduino.h>
#include <arduino_lmic_hal_boards.h>

// leuville-arduino-utilities
#include <misc-util.h>
#include <ISRWrapper.h>
#include <ISRTimer.h>
#include <EnergyController.h>
#include <JobRegister.h>

namespace lstl = leuville::simple_template_library;
namespace lora = leuville::lora;

using namespace lstl;
using namespace lora;

// LoRaWAN configuration: see OTAAId in LMICWrapper.h
#include "lora-common-defs.h"

#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
USBPrinter<Serial_> console(LMIC_PRINTF_TO);
#endif

/*
 * LoraWan endnode with:
 * - a timer to trigger a PING message on a regular basis
 * - a callback set on button connected to a pin, which triggers a BUTTON message
 * - standby mode capacity
 */
using Button1 = ISRWrapper<DEVICE_BUTTON1_PIN>;
using EnergyCtrl = EnergyController<VOLTAGE_MIN,VOLTAGE_MAX>;

template <typename T>
class EndNodeBase : public T, 
					protected ISRTimer, 
					protected Button1,
					protected EnergyCtrl
{

protected:

	/*
	 * Redefines EnergyController::defineGetVoltage()
	 */
	#if defined(ARDUINO_SAMD_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0)
	virtual std::function<double(void)> defineGetVoltage() override {
		return []() -> double { return analogRead(A7) * 2 * 3.3 / 1.023; };
	}
	#endif

	/*
	 * Jobs for LMIC event callbacks
	 *
	 * USER1, USER2, USER3 are free for additional user-defined jobs
	 */
	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
	enum JOB { BUTTON, TIMEOUT, JOIN, TXCOMPLETE, USER1, USER2, USER3, _COUNT };
	#else
	enum JOB { BUTTON, TIMEOUT, JOIN, USER1, USER2, USER3, _COUNT };
	#endif
	JobRegister<EndNodeBase,JOB::_COUNT> _callbacks;

public:

	EndNodeBase(const lmic_pinmap &pinmap): EndNodeBase(&pinmap) {}

	EndNodeBase(const lmic_pinmap *pinmap = Arduino_LMIC::GetPinmap_ThisBoard()): 
		T(pinmap),
		ISRTimer(60 * DEVICE_MEASURE_DELAY, true),
		Button1(INPUT_PULLUP, LOW)
	{
		_callbacks.define(BUTTON, 		this, &EndNodeBase::buttonJob);
		_callbacks.define(TIMEOUT, 		this, &EndNodeBase::timeoutJob);
		_callbacks.define(JOIN, 		this, &EndNodeBase::joinJob);
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 1
		_callbacks.define(TXCOMPLETE, 	this, &EndNodeBase::txCompleteJob);
		#endif
	}

	/*
	 * delegates begin() to each sub-component and join
	 *
	 * ORDER is important
	 * If LMIC_USE_INTERRUPTS is enabled, LMIC must be initialized after other interruptions
	 * 
	 * Do not forget Wire.begin() if I2C devices connected
	 */
	virtual void begin(const OTAAId& id, u4_t network, bool adr = true) override {
		EnergyCtrl::begin();
		Button1::begin();
		ISRTimer::begin(); 
		T::begin(id, network, adr);

		T::startJoining();
	}

	/*
	 * Configure LoRaWan network (channels, power, adr, ...)
	 */
	virtual void initLMIC(u4_t network = 0, bool adr = true) override {
		T::initLMIC(network, adr);
		configureNetwork(static_cast<Network>(network), adr);
	}

	/*
	 * Button ISR
	 * job done by sending a LMIC callback
	 * see completeJob()
	 */
	virtual void ISR_callback(uint8_t pin) override {
		T::setCallback(_callbacks[BUTTON]);
	}

	/*
	 * Timer ISR
	 * job done by sending a LMIC callback
	 * see completeJob()
	 */
	virtual uint32_t ISR_timeout() override {
		T::setCallback(_callbacks[TIMEOUT]);
		return 60 * DEVICE_MEASURE_DELAY;
	}

	/*
	 * Job done on join/unjoin
	 * see completeJob()
	 */
	virtual void joined(bool ok) override {
		if (ok) {
			Button1::enable();
			ISRTimer::enable();
			T::setCallback(_callbacks[JOIN]);
		} else {
			Button1::disable();
			ISRTimer::disable();
			for (auto & job : _callbacks) {
				T::unsetCallback(job);
			}
		}
	}

	/*
	 * Updates the system time
	 */
	#if defined(LMIC_ENABLE_DeviceTimeReq)
	virtual void updateSystemTime(uint32_t newTime) override {
		ISRTimer::setEpoch(newTime);
    	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		RTCZero & rtc = ISRTimer::getRTC();
		console.println("----------------------------------------------------------");
		console.print("heure network=",	rtc.getHours()); 
		console.print(":", rtc.getMinutes());
		console.println(":", rtc.getSeconds());
		console.println("----------------------------------------------------------");
    	#endif
	}
	virtual bool isSystemTimeSynced() override {
    	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		console.println("----------------------------------------------------------");
		console.println("system time age=", systemTimeAge()); 
		console.println("----------------------------------------------------------");
		#endif
		return T::systemTimeAge() < SYSTEM_TIME_MAX_AGE;
	}

	#endif

	/*
	 * Handle LMIC callbacks
	 */
	virtual void completeJob(osjob_t* job) override {
		_callbacks[job]();
	}

	/*
	 * TO OVERRIDE
	 */
	virtual void buttonJob() = 0;

	/*
	 * TO OVERRIDE
	 */
	virtual void timeoutJob() = 0;

	virtual void joinJob() {
		LoRaWanSessionKeys keys = T::getSessionKeys();
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		console.println("----------------------------------------------------------");
		console.println("netId: ", keys._netId, HEX);
		console.println("devAddr: ", keys._devAddr, HEX);
		console.print("nwkSKey: "); console.printHex(keys._nwkSKey, arrayCapacity(keys._nwkSKey));
		console.print("appSKey: "); console.printHex(keys._appSKey, arrayCapacity(keys._appSKey));
		console.println("----------------------------------------------------------");
		#endif
		postJoinSetup(keys._netId);
	}

	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 1

	virtual void txCompleteJob() {
		console.println("----------------------------------------------------------");
		console.println("FIFO size: ", _messages.size());
		console.println("----------------------------------------------------------");
	}

	/*
	 * LMIC callback called on TX_COMPLETE event
	 */
	virtual bool isTxCompleted(const UpstreamMessage & message) override {
		T::setCallback(_callbacks[TXCOMPLETE]);
		console.println("----------------------------------------------------------");
		console.print("isTxCompleted "); 
		console.print(" / len: ", message._len);
		console.print(" / ackRequest: ", message._ackRequested);
		console.print(" / txrxFlags: ", message._txrxFlags);
		console.println(" / lmicError: ", (int32_t)message._lmicTxError);
		console.println("----------------------------------------------------------");
		return T::isTxCompleted(message);
	}	
	
	#endif

	/*
	 * Activates standby mode
	 */
	virtual void standby() {
		ISRTimer::standbyMode();
	}

};

