/*********************************************************************
 * TrackController - Hacking your Märklin
 *
 * Original version Railuino Copyright (C) 2012 Joerg Pleumann
 * Adapted by Gerard Harkema januari 2024
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * LICENSE file for more details.
 */

#ifndef railuino__h
#define railuino__h

#include <Arduino.h>
#include <Printable.h>
#include "mcp2515.hpp"

// ===================================================================


// ===================================================================
// === Common definitions ============================================
// ===================================================================

/**
 * Version of Railuino library and required connection box software.
 */
#define TRACK_CONTROL_VERSION 0x0101 // 1.01
#define TRACKBOX_VERSION 0x0127 // 1.39

/**
 * Constants for protocol base addresses.
 */
#define ADDR_MM2     0x0000 // MM2 locomotive
#define ADDR_MM2_MAX 0x03ff // MM2 locomotive
#define ADDR_SX1     0x0800 // Selectrix (old) locomotive
#define ADDR_MFX     0x4000 // MFX locomotive
#define ADDR_MFX_MAX 0x7fff // MFX locomotive
#define ADDR_SX2     0x8000 // Selectrix (new) locomotive
#define ADDR_SX2_MAX 0xbfff // Selectrix (new) locomotive
#define ADDR_DCC     0xC000 // DCC locomotive
#define ADDR_DCC_MAX 0xffff // DCC locomotive
#define ADDR_ACC_SX1 0x2000 // Selectrix (old) magnetic accessory
#define ADDR_ACC_MM2 0x2FFF // MM2 magnetic accessory
#define ADDR_ACC_DCC 0x3800 // DCC magnetic accessory

/**
 * Constants for classic MM2 Delta addresses.
 */
#define DELTA1       78
#define DELTA2       72
#define DELTA3       60
#define DELTA4       24

/**
 * Constants for locomotive directions.
 */
#define DIR_CURRENT  0
#define DIR_FORWARD  1
#define DIR_REVERSE  2
#define DIR_CHANGE   3

/**
 * Constants for accessory states including some aliases.
 */
#define ACC_OFF      0
#define ACC_ROUND    0
#define ACC_RED      0
#define ACC_RIGHT    0
#define ACC_HP0      0

#define ACC_ON       1
#define ACC_GREEN    1
#define ACC_STRAIGHT 1
#define ACC_HP1      1

#define ACC_YELLOW   2
#define ACC_LEFT     2
#define ACC_HP2      2

#define ACC_WHITE    3
#define ACC_SH0      3



// Commando's


#define SYSTEM_BEFEHL         0x00
#define DISCOVERY             0x01
#define MFX_BIND              0x02
#define MFX_VERIFY            0x03
#define LOC_GESCHWINDIGHEID   0x04
#define LOC_RICHTUNG          0x05
#define LOC_FUNCTION          0x06
#define READ_CONFIG           0x07
#define WRITE_CONFIG          0x08
#define ZUBEHOR_SCHALTEN      0x0b
#define ZUBEHOR_CONFIG        0x0c
#define S88_POLLING           0x10
#define S88_EVENT             0x11
#define SX1_EVENT             0x12
#define PING                  0x18
#define STATUSDATEN           0x1d


// Subcomando's SYSTEM_BEFEHL
#define SYSTEM_STOP       0x00
#define SYSTEM_GO         0x01
#define SYSTEM_HALT       0x02
#define LOC_NOTHALT       0x03
#define LOC_ZYCLUS_STOP   0x04
#define NUEANMELDERZAHLER 0x09
#define UBERLAST          0x0a
#define STATUS            0x0b

#define STATUS_CHANNEL_CURRENT      1
#define STATUS_CHANNEL_VOLTAGE      3
#define STATUS_CHANNEL_TEMPERATURE  4

// Memory location defines

#define TURNOUT_BASE_ADDRESS    0x3000
#define TURNOUT_END_ADDRESS     0x3fff

#define NUMBER_OF_MM1_FUNCTIONS         4
#define NUMBER_OF_MM2_FUNCTIONS         4
#define NUMBER_OF_DCC_FUNCTIONS         16
#define NUMBER_OF_MFX_FUNCTIONS         32



/**
 * Represents a message going through the Marklin CAN bus. More or
 * less a beautified version of the real CAN message. You normally
 * don't need to use this unless you want to experiment with the
 * protocol or extend the library. See the Marklin protocol
 * documentation for details. The TrackMessage is a Printable, so
 * it can be directly used in Serial.println(), for instance. It
 * can also be converted from a String.
 */
class TrackMessage : public Printable {

  public:

  /**
   * The command number.
   */
  byte command;
  
  /**
   * The hash that is used for avoiding device/message collisions.
   */
  word hash;

  /**
   * Whether this is a response to a request.
   */
  boolean response;

  /**
   * The number of data bytes in the payload.
   */
  byte length;

  /**
   * The actual message data bytes.
   */
  byte data[8];

  /**
   * Clears the message, setting all values to zero. Provides for
   * easy recycling of TrackMessage objects.
   */
  void clear();

  /**
   * Prints the message to the given Print object, which could be a
   * Serial object, for instance. The message format looks like this
   *
   * HHHH R CC L DD DD DD DD DD DD DD DD
   *
   * with all numbers being hexadecimals and the data bytes being
   * optional beyond what the message length specifies. Exactly one
   * whitespace is inserted between different fields as a separator.
   */
  virtual size_t printTo(Print &p) const;

  /**
   * Parses the message from the given String. Returns true on
   * success, false otherwise. The message must have exactly the
   * format that printTo creates. This includes each and every
   * whitespace. If the parsing fails the state of the object is
   * undefined afterwards, and a clear() is recommended.
   */
  boolean parseFrom(String &s);

};

// ===================================================================
// === TrackController ===============================================
// ===================================================================

/**
 * Controls things on and connected to the track: locomotives,
 * turnouts and other accessories. While there are some low-level
 * methods for dealing with messages, you will normally want to use
 * the high-level methods that wrap most of the the nasty protocol
 * details. When addressing something, you have to tell the system the
 * type of address (or decoder) you are using by adding the proper
 * protocol base address. For instance, DCC locomotive 42 is properly
 * addressed as ADDR_DCC + 42.
 */
class TrackController {

    private:

	/**
	 * Stores the hash of our controller. This must not conflict with
	 * hashes of other devices in the setup (most likely the MS2 and
	 * the connector box).
	 */
	word mHash;

	/**
	 * Stores the debug flag. When debugging is on, all outgoing and
	 * incoming messages are printed to the Serial console.
	 */
	boolean mDebug;
	
	/**
	 * Holds the loopback flag. When loopback is on, messages are
	 * reflected by the CAN controller. No external communication
	 * takes place. This is helpful for some test cases.
	 */
	boolean mLoopback;

    uint16_t message_count = 0;
    
#ifdef ESP32
    TaskHandle_t enqueue_task_h;
#endif

	/**
	 * Generates a new hash and makes sure it does not conflict
	 * with those of other devices in the setup.
	 */
	void generateHash();

    public:

	/**
	 * Creates a new TrackController with default values. This should
	 * be fine for most use cases. Further configuration can be done
	 * by using the init() method.
	 */
    TrackController();

	/**
	 * Creates a new TrackController with the given hash and debugging
	 * flag. A zero hash will result in a unique hash begin generated.
	 */
    TrackController(word hash, boolean debug);

    /**
     * Is called when a TrackController is being destroyed. Does the
     * necessary cleanup. No need to call this manually.
     */
    ~TrackController();

    /**
     * Initializes the TrackController with the given values. This
     * should be called before begin, otherwise it will not take
     * effect. A zero hash will result in a unique hash begin
     * generated.
     */
    void init(word hash, boolean debug, boolean loopback);


    void printMessageHeader();

    /**
     * Queries the hash used by the TrackController.
     */
    word getHash();

    /**
     * Reflects whether the TrackController is in debug mode,
     * where all messages are dumped to the Serial console.
     */
    boolean isDebug();

    /**
     * Reflects whether the TrackController is in debug mode,
     * where all messages are reflected by the CAN controller.
     */
    boolean isLoopback();

    /**
     * Sends a message and reports true on success. Internal method.
     * Normally you don't want to use this, but the more convenient
     * methods below instead.
     */
    boolean sendMessage(TrackMessage &message);

    /**
     * Receives an arbitrary message, if available, and reports true
     * on success. Does not block. Internal method. Normally you
     * don't want to use this, but the more convenient methods below
     * instead.
     */
    boolean receiveMessage(TrackMessage &message);

    boolean isCanbusError();

    /**
     * Sends a message and waits for the corresponding response,
     * returning true on success. Blocks until either a message with
     * the same command ID and the response marker arrives or the
     * timeout (in ms) expires. All non-matching messages are
     * skipped. Internal method. Normally you don't want to use this,
     * but the more convenient methods below instead. 'out' and 'in'
     * may be the same object.
     */
    boolean exchangeMessage(TrackMessage &out, TrackMessage &in,  word timeout);

    /**
     * Initializes the CAN hardware and starts receiving CAN
     * messages. CAN messages are put into an internal buffer of
     * limited size, so they don't get lost, but you have to take
     * care of them in time. Otherwise the buffer might overflow.
     */
    void begin();

    /**
     * Stops receiving messages from the CAN hardware. Clears
     * the internal buffer.
     */
    void end();

    /**
     * Controls power on the track. When passing false, all
     * locomotives will stop, but remember their previous directions
     * and speeds. When passing true, all locomotives will regain
     * their old directions and speeds. The system starts in
     * stopped mode in order to avoid accidents. The return value
     * reflects whether the call was successful.
     */
    boolean setPower(boolean power);
#if 0
    boolean getPower(boolean *power)
#endif
    
    /**
     * Sets the direction of the given locomotive. Valid directions
     * are those specified by the DIR_* constants. The return value
     * reflects whether the call was successful.
     */
    boolean setLocoDirection(word address, byte direction);

    /**
     * Toggles the direction of the given locomotive. This normally
     * includes a full stop.
     */
    boolean toggleLocoDirection(word address);
    
    /**
     * Sets the speed of the given locomotive. Valid speeds are
     * 0 to 1023 (inclusive), though the connector box will limit
     * all speeds beyond 1000 to 1000. The return value reflects
     * whether the call was successful.
     */
    boolean setLocoSpeed(word address, word speed);

    /**
     * Increases the speed of the given locomotive by 1/14th
     * of the maximum speed.
     */
    boolean accelerateLoco(word address);

    /**
     * Decreases the speed of the given locomotive by 1/14th
     * of the maximum speed.
     */
    boolean decelerateLoco(word address);

    /**
     * Sets the given function of the given locomotive (or simply a
     * function decoder). Valid functions are 0 to 31, with 0
     * normally denoting the head/backlight. Valid values are, again,
     * 0 ("off") to 31, although not all protocols support values
     * beyond 1 (which then means "on").  The return value reflects
     * whether the call was successful.
     */
    boolean setLocoFunction(word address, byte function, byte power);
    
    /**
     * Toggles the given function of the given locomotive. Valid
     * functions are 0 to 31, with 0 normally denoting the
     * head/backlight.
     */
    boolean toggleLocoFunction(word address, byte function);
    
    /**
     * Switches the given magnetic accessory. Valid position values
     * are those denoted by the ACC_* constants. Valid power values
     * are 0 ("off") to 31 (inclusive) although not all protocols
     * support values beyond 1 (which then means "on"). The final
     * parameter specifies the time (in ms) for which the accessory
     * will be active. A time of 0 means the accessory will only be
     * switched on. Some magnetic accessories must not be active for
     * too long, because they might burn out. A good timeout for
     * Marklin turnouts seems to be 20 ms. The return value reflects
     * whether the call was successful.
     */
    boolean setAccessory(word address, byte position, byte power, word time);
    
    /**
     * Switches a turnout. This is actually a convenience function
     * around setAccessory() that uses default values for some
     * parameters. The return value reflects whether the call was
     * successful.
     */
    boolean setTurnout(word address, boolean straight);
    
    /**
     * Queries the direction of the given locomotive and writes it
     * into the referenced byte. The return value indicates whether
     * the call was successful and the direction is valid.
     */
    boolean getLocoDirection(word address, byte *direction);
    
    /**
     * Queries the speed of the given locomotive and writes it
     * into the referenced byte. The return value indicates whether
     * the call was successful and the speed is valid.
     */
    boolean getLocoSpeed(word address, word *speed);

    /**
     * Queries the given function of the given locomotive and writes
     * it into the referenced byte. The return value indicates
     * whether the call was successful and the power is valid. Note
     * that the call will not reflect the original power value sent
     * to the function, but only 0 ("off") or 1 ("on").
     */
    boolean getLocoFunction(word address, byte function, byte *power);
    
    /**
     * Queries the given magnetic accessory's state and and writes
     * it into the referenced bytes. The return value indicates
     * whether the call was successful and the bytes are valid. Note
     * that the call will not reflect the original power value sent
     * to the function, but only 0 ("off") or 1 ("on").
     */
    boolean getAccessory(word address, byte *position, byte *power);

    /**
     * Queries a turnout state. This is actually a convenience
     * function around getAccessory() that uses default values for
     * some parameters.
     */
    boolean getTurnout(word address, boolean *straight);

    /**
     * Queries the current on the track. 
     */
    float getCurrent();

    /**
     * Queries the voltage on the track. 
     */
    float getVoltage();

    /**
     * Queries the temperature of the controller
     */
    float getTemperature();

    /**
     * Writes the given value to the given config number of the given
     * locomotive.  The return value reflects whether the call was
     * successful.
     */
    boolean writeConfig(word address, word number, byte value);
    
    /**
     * Reads the given config number of the given locomotive into the
     * given value.
     */
    boolean readConfig(word address, word number, byte *value);
    
    /**
     * Queries the software version of the track format processor.
     */
    boolean getVersion(byte *high, byte *low); 

};



#endif
