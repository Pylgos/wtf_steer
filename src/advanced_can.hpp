#ifndef ADVANCED_CAN_HPP
#define ADVANCED_CAN_HPP

#include "PeripheralPins.h"
#include "drivers/CAN.h"
#include "hal/can_api.h"
#include "interfaces/InterfaceCAN.h"
#include "platform/Callback.h"
#include "platform/mbed_error.h"
#include "platform/mbed_power_mgmt.h"


class AdvancedCAN : public mbed::interface::can {
 public:
  AdvancedCAN() {}

  virtual ~AdvancedCAN() {
    // Detaching interrupts releases the sleep lock if it was locked
    for(int irq = 0; irq < IrqType::IrqCnt; irq++) {
      attach(nullptr, (IrqType)irq);
    }
    can_irq_free(&_can);
    can_free(&_can);
  };

  int init(PinName rd, PinName td, int hz) {
    CANName can_rd = (CANName)pinmap_peripheral(rd, PinMap_CAN_RD);
    CANName can_td = (CANName)pinmap_peripheral(td, PinMap_CAN_TD);
    int peripheral = (int)pinmap_merge(can_rd, can_td);

    int function_rd = (int)pinmap_find_function(rd, PinMap_CAN_RD);
    int function_td = (int)pinmap_find_function(td, PinMap_CAN_TD);

    const can_pinmap_t pinmap = {peripheral, rd, function_rd, td, function_td};

    MBED_ASSERT((int)pinmap.peripheral != NC);

    if(pinmap.peripheral == CAN_1) {
      __HAL_RCC_CAN1_CLK_ENABLE();
      _can.index = 0;
    }
// #if defined(CAN2_BASE) && (CAN_NUM > 1)
    else if(pinmap.peripheral == CAN_2) {
      __HAL_RCC_CAN1_CLK_ENABLE();  // needed to set filters
      __HAL_RCC_CAN2_CLK_ENABLE();
      _can.index = 1;
    }
// #endif
#if defined(CAN3_BASE) && (CAN_NUM > 2)
    else if(pinmap->peripheral == CAN_3) {
      __HAL_RCC_CAN3_CLK_ENABLE();
      _can.index = 2;
    }
#endif
    else {
      return 0;
    }

    // Configure CAN pins
    pin_function(pinmap.rd_pin, pinmap.rd_function);
    pin_function(pinmap.td_pin, pinmap.td_function);
    // Add pull-ups
    pin_mode(pinmap.rd_pin, PullUp);
    pin_mode(pinmap.td_pin, PullUp);

    /*  Use default values for rist init */
    _can.CanHandle.Instance = (CAN_TypeDef *)pinmap.peripheral;
    _can.CanHandle.Init.TTCM = DISABLE;
    _can.CanHandle.Init.ABOM = ENABLE;
    _can.CanHandle.Init.AWUM = DISABLE;
    _can.CanHandle.Init.NART = DISABLE;
    _can.CanHandle.Init.RFLM = DISABLE;
    _can.CanHandle.Init.TXFP = DISABLE;
    _can.CanHandle.Init.Mode = CAN_MODE_NORMAL;
    _can.CanHandle.Init.SJW = CAN_SJW_1TQ;
    _can.CanHandle.Init.BS1 = CAN_BS1_6TQ;
    _can.CanHandle.Init.BS2 = CAN_BS2_8TQ;
    _can.CanHandle.Init.Prescaler = 2;

    /*  Store frequency to be restored in case of reset */
    _can.hz = hz;

    if(HAL_CAN_Init(&_can.CanHandle) != HAL_OK) {
      return 0;
      // error("Cannot initialize CAN");
    }

    // Set initial CAN frequency to specified frequency
    if(can_frequency(&_can, _can.hz) != 1) {
      return 0;
      // error("Can frequency could not be set\n");
    }

    /* Bits 27:14 are available for dual CAN configuration and are reserved for
       single CAN configuration: */
#if defined(CAN3_BASE) && (CAN_NUM > 2)
    uint32_t filter_number = (pinmap->peripheral == CAN_1 || pinmap->peripheral == CAN_3) ? 0 : 14;
#else
    uint32_t filter_number = (pinmap.peripheral == CAN_1) ? 0 : 14;
#endif
    can_filter(&_can, 0, 0, CANStandard, filter_number);

    can_irq_init(&_can, (&AdvancedCAN::_irq_handler), reinterpret_cast<uintptr_t>(this));
    _is_opened = true;
    return 1;
  }

  bool isOpened() {
    return _is_opened;
  }

  /** Set the frequency of the CAN interface
     *
     *  @param hz The bus frequency in hertz
     *
     *  @returns
     *    1 if successful,
     *    0 otherwise
     */
  int frequency(int hz) {
    return can_frequency(&_can, hz);
  }

  /** Write a CANMessage to the bus.
     *
     *  @param msg The CANMessage to write.
     *
     *  @returns
     *    0 if write failed,
     *    1 if write was successful
     */
  int write(mbed::CANMessage msg) {
    return can_write(&_can, msg, 0);
  }

  /** Read a CANMessage from the bus.
     *
     *  @param msg A CANMessage to read to.
     *  @param handle message filter handle (0 for any message)
     *
     *  @returns
     *    0 if no message arrived,
     *    1 if message arrived
     */
  int read(mbed::CANMessage &msg, int handle = 0) {
    int ret = can_read(&_can, &msg, handle);
    if (msg.len > 8) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_CAN, MBED_ERROR_CODE_READ_FAILED), "Read tried to write more than 8 bytes");
    }
    return ret;
  }

  /** Reset CAN interface.
     *
     * To use after error overflow.
     */
  void reset() {
    can_reset(&_can);
  }

  /** Puts or removes the CAN interface into silent monitoring mode
     *
     *  @param silent boolean indicating whether to go into silent mode or not
     */
  void monitor(bool silent) {
    can_monitor(&_can, (silent) ? 1 : 0);
  }

  /** Change CAN operation to the specified mode
     *
     *  @param mode The new operation mode (CAN::Normal, CAN::Silent, CAN::LocalTest, CAN::GlobalTest, CAN::SilentTest)
     *
     *  @returns
     *    0 if mode change failed or unsupported,
     *    1 if mode change was successful
     */
  int mode(Mode mode);

  /** Filter out incoming messages
     *
     *  @param id the id to filter on
     *  @param mask the mask applied to the id
     *  @param format format to filter on (Default CANAny)
     *  @param handle message filter handle (Optional)
     *
     *  @returns
     *    0 if filter change failed or unsupported,
     *    new filter handle if successful
     */
  int filter(unsigned int id, unsigned int mask, CANFormat format = CANAny, int handle = 0);

  /**  Detects read errors - Used to detect read overflow errors.
     *
     *  @returns number of read errors
     */
  unsigned char rderror() {
    return can_rderror(&_can);
  }

  /** Detects write errors - Used to detect write overflow errors.
     *
     *  @returns number of write errors
     */
  unsigned char tderror() {
    return can_tderror(&_can);
  }

  /** Attach a function to call whenever a CAN frame received interrupt is
     *  generated.
     *
     *  This function locks the deep sleep while a callback is attached
     *
     *  @param func A pointer to a void function, or 0 to set as none
     *  @param type Which CAN interrupt to attach the member function to (CAN::RxIrq for message received, CAN::TxIrq for transmitted or aborted, CAN::EwIrq for error warning, CAN::DoIrq for data overrun, CAN::WuIrq for wake-up, CAN::EpIrq for error passive, CAN::AlIrq for arbitration lost, CAN::BeIrq for bus error)
     */
  void attach(mbed::Callback<void()> func, IrqType type = IrqType::RxIrq) {
    if (func) {
        // lock deep sleep only the first time
        if (!_irq[(CanIrqType)type]) {
            sleep_manager_lock_deep_sleep();
        }
        _irq[(CanIrqType)type] = func;
        can_irq_set(&_can, (CanIrqType)type, 1);
    } else {
        // unlock deep sleep only the first time
        if (_irq[(CanIrqType)type]) {
            sleep_manager_unlock_deep_sleep();
        }
        _irq[(CanIrqType)type] = nullptr;
        can_irq_set(&_can, (CanIrqType)type, 0);
    }
  }

  static void _irq_handler(uintptr_t context, CanIrqType type) {
    AdvancedCAN *handler = reinterpret_cast<AdvancedCAN *>(context);
    if (handler->_irq[type]) {
        handler->_irq[type].call();
    }
  }

#if !defined(DOXYGEN_ONLY)
 protected:
  bool _is_opened = false;
  can_t _can;
  mbed::Callback<void()> _irq[IrqType::IrqCnt];
#endif
};

#endif
