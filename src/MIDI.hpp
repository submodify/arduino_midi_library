/*!
 *  @file       MIDI.hpp
 *  Project     Arduino MIDI Library
 *  @brief      MIDI Library for the Arduino - Inline implementations
 *  @author     Francois Best, lathoub
 *  @date       24/02/11
 *  @license    MIT - Copyright (c) 2015 Francois Best
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

BEGIN_MIDI_NAMESPACE

/// \brief Constructor for MidiInterface.
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>::MidiInterface(Transport& inTransport)
    : mTransport(inTransport)
    , mInputChannel(0)
    , mRunningStatus_RX(InvalidType)
    , mRunningStatus_TX(InvalidType)
    , mPendingMessageExpectedLength(0)
    , mPendingMessageIndex(0)
    , mCurrentRpnNumber(0xffff)
    , mCurrentNrpnNumber(0xffff)
    , mThruActivated(true)
    , mThruFilterMode(Thru::Full)
    , mLastMessageSentTime(0)
    , mLastMessageReceivedTime(0)
    , mSenderActiveSensingPeriodicity(0)
    , mReceiverActiveSensingActivated(false)
    , mLastError(0)
{
    mSenderActiveSensingPeriodicity = Settings::SenderActiveSensingPeriodicity;
}

/*! \brief Destructor for MidiInterface.

 This is not really useful for the Arduino, as it is never called...
 */
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>::~MidiInterface()
{
}

// -----------------------------------------------------------------------------

/*! \brief Call the begin method in the setup() function of the Arduino.

 All parameters are set to their default values:
 - Input channel set to 1 if no value is specified
 - Full thru mirroring
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::begin(Channel inChannel)
{
    // Initialise the Transport layer
    mTransport.begin();

    mInputChannel = inChannel;
    mRunningStatus_TX = InvalidType;
    mRunningStatus_RX = InvalidType;

    mPendingMessageIndex = 0;
    mPendingMessageExpectedLength = 0;

    mCurrentRpnNumber  = 0xffff;
    mCurrentNrpnNumber = 0xffff;

    mLastMessageSentTime = Platform::now();

    mMessage.valid   = false;
    mMessage.type    = InvalidType;
    mMessage.channel = 0;
    mMessage.data1   = 0;
    mMessage.data2   = 0;
    mMessage.length  = 0;

    mThruFilterMode = Thru::Full;
    mThruActivated  = mTransport.thruActivated;

    return *this;
}

// -----------------------------------------------------------------------------
//                                 Output
// -----------------------------------------------------------------------------

/*! \addtogroup output
 @{
 */

/*! \brief Send a MIDI message.
\param inMessage    The message

 This method is used when you want to send a Message that has not been constructed
 by the library, but by an external source.
 This method does *not* check against any of the constraints.
 Typically this function is use by MIDI Bridges taking MIDI messages and passing
 them thru.
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::send(const MidiMessage& inMessage)
{
    if (!inMessage.valid)
        return *this;

    if (mTransport.beginTransmission(inMessage.type))
    {
        if (inMessage.isSystemRealTime())
        {
            mTransport.write(inMessage.type);
        } else if (inMessage.isChannelMessage())
        {
            const StatusByte status = getStatus(inMessage.type, inMessage.channel);
            mTransport.write(status);
            if (inMessage.length > 1) mTransport.write(inMessage.data1);
            if (inMessage.length > 2) mTransport.write(inMessage.data2);
        } else if (inMessage.type == MidiType::SystemExclusive)
        {
            const unsigned size = inMessage.getSysExSize();
            for (size_t i = 0; i < size; i++)
                mTransport.write(inMessage.sysexArray[i]);
        } else // at this point, it it assumed to be a system common message
        {
            mTransport.write(inMessage.type);
            if (inMessage.length > 1) mTransport.write(inMessage.data1);
            if (inMessage.length > 2) mTransport.write(inMessage.data2);
        }
    }
    mTransport.endTransmission();
    updateLastSentTime();

    return *this;
}


/*! \brief Generate and send a MIDI message from the values given.
 \param inType    The message type (see type defines for reference)
 \param inData1   The first data byte.
 \param inData2   The second data byte (if the message contains only 1 data byte,
 set this one to 0).
 \param inChannel The output channel on which the message will be sent
 (values from 1 to 16). Note: you cannot send to OMNI.

 This is an internal method, use it only if you need to send raw data
 from your code, at your own risks.
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::send(MidiType inType,
                                               DataByte inData1,
                                               DataByte inData2,
                                               Channel inChannel)
{
    if (inType <= PitchBend)  // Channel messages
    {
        // Then test if channel is valid
        if (inChannel >= MIDI_CHANNEL_OFF  ||
            inChannel == MIDI_CHANNEL_OMNI ||
            inType < 0x80)
        {
            return *this; // Don't send anything
        }
        // Protection: remove MSBs on data
        inData1 &= 0x7f;
        inData2 &= 0x7f;

        const StatusByte status = getStatus(inType, inChannel);

        if (mTransport.beginTransmission(inType))
        {
            if (Settings::UseRunningStatus)
            {
                if (mRunningStatus_TX != status)
                {
                    // New message, memorise and send header
                    mRunningStatus_TX = status;
                    mTransport.write(mRunningStatus_TX);
                }
            }
            else
            {
                // Don't care about running status, send the status byte.
                mTransport.write(status);
            }

            // Then send data
            mTransport.write(inData1);
            if (inType != ProgramChange && inType != AfterTouchChannel)
            {
                mTransport.write(inData2);
            }

            mTransport.endTransmission();
            updateLastSentTime();
        }
    }
    else if (inType >= Clock && inType <= SystemReset)
    {
        sendRealTime(inType); // System Real-time and 1 byte.
    }

    return *this;
}

// -----------------------------------------------------------------------------

/*! \brief Send a Note On message
 \param inNoteNumber  Pitch value in the MIDI format (0 to 127).
 \param inVelocity    Note attack velocity (0 to 127). A NoteOn with 0 velocity
 is considered as a NoteOff.
 \param inChannel     The channel on which the message will be sent (1 to 16).

 Take a look at the values, names and frequencies of notes here:
 http://www.phys.unsw.edu.au/jw/notes.html
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendNoteOn(DataByte inNoteNumber,
                                                     DataByte inVelocity,
                                                     Channel inChannel)
{
    return send(NoteOn, inNoteNumber, inVelocity, inChannel);
}

/*! \brief Send a Note Off message
 \param inNoteNumber  Pitch value in the MIDI format (0 to 127).
 \param inVelocity    Release velocity (0 to 127).
 \param inChannel     The channel on which the message will be sent (1 to 16).

 Note: you can send NoteOn with zero velocity to make a NoteOff, this is based
 on the Running Status principle, to avoid sending status messages and thus
 sending only NoteOn data. sendNoteOff will always send a real NoteOff message.
 Take a look at the values, names and frequencies of notes here:
 http://www.phys.unsw.edu.au/jw/notes.html
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendNoteOff(DataByte inNoteNumber,
                                                      DataByte inVelocity,
                                                      Channel inChannel)
{
    return send(NoteOff, inNoteNumber, inVelocity, inChannel);
}

/*! \brief Send a Program Change message
 \param inProgramNumber The Program to select (0 to 127).
 \param inChannel       The channel on which the message will be sent (1 to 16).
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendProgramChange(DataByte inProgramNumber,
                                                            Channel inChannel)
{
    return send(ProgramChange, inProgramNumber, 0, inChannel);
}

/*! \brief Send a Control Change message
 \param inControlNumber The controller number (0 to 127).
 \param inControlValue  The value for the specified controller (0 to 127).
 \param inChannel       The channel on which the message will be sent (1 to 16).
 @see MidiControlChangeNumber
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendControlChange(DataByte inControlNumber,
                                                            DataByte inControlValue,
                                                            Channel inChannel)
{
    return send(ControlChange, inControlNumber, inControlValue, inChannel);
}

/*! \brief Send a Polyphonic AfterTouch message (applies to a specified note)
 \param inNoteNumber  The note to apply AfterTouch to (0 to 127).
 \param inPressure    The amount of AfterTouch to apply (0 to 127).
 \param inChannel     The channel on which the message will be sent (1 to 16).
 Note: this method is deprecated and will be removed in a future revision of the
 library, @see sendAfterTouch to send polyphonic and monophonic AfterTouch messages.
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendPolyPressure(DataByte inNoteNumber,
                                                           DataByte inPressure,
                                                           Channel inChannel)
{
    return send(AfterTouchPoly, inNoteNumber, inPressure, inChannel);
}

/*! \brief Send a MonoPhonic AfterTouch message (applies to all notes)
 \param inPressure    The amount of AfterTouch to apply to all notes.
 \param inChannel     The channel on which the message will be sent (1 to 16).
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendAfterTouch(DataByte inPressure,
                                                         Channel inChannel)
{
    return send(AfterTouchChannel, inPressure, 0, inChannel);
}

/*! \brief Send a Polyphonic AfterTouch message (applies to a specified note)
 \param inNoteNumber  The note to apply AfterTouch to (0 to 127).
 \param inPressure    The amount of AfterTouch to apply (0 to 127).
 \param inChannel     The channel on which the message will be sent (1 to 16).
 @see Replaces sendPolyPressure (which is now deprecated).
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendAfterTouch(DataByte inNoteNumber,
                                                         DataByte inPressure,
                                                         Channel inChannel)
{
    return send(AfterTouchPoly, inNoteNumber, inPressure, inChannel);
}

/*! \brief Send a Pitch Bend message using a signed integer value.
 \param inPitchValue  The amount of bend to send (in a signed integer format),
 between MIDI_PITCHBEND_MIN and MIDI_PITCHBEND_MAX,
 center value is 0.
 \param inChannel     The channel on which the message will be sent (1 to 16).
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendPitchBend(int inPitchValue,
                                                        Channel inChannel)
{
    const unsigned bend = unsigned(inPitchValue - int(MIDI_PITCHBEND_MIN));
    return send(PitchBend, (bend & 0x7f), (bend >> 7) & 0x7f, inChannel);
}


/*! \brief Send a Pitch Bend message using a floating point value.
 \param inPitchValue  The amount of bend to send (in a floating point format),
 between -1.0f (maximum downwards bend)
 and +1.0f (max upwards bend), center value is 0.0f.
 \param inChannel     The channel on which the message will be sent (1 to 16).
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendPitchBend(double inPitchValue,
                                                        Channel inChannel)
{
    const int scale = inPitchValue > 0.0 ? MIDI_PITCHBEND_MAX : - MIDI_PITCHBEND_MIN;
    const int value = int(inPitchValue * double(scale));
    return sendPitchBend(value, inChannel);
}

/*! \brief Generate and send a System Exclusive frame.
 \param inLength  The size of the array to send
 \param inArray   The byte array containing the data to send
 \param inArrayContainsBoundaries When set to 'true', 0xf0 & 0xf7 bytes
 (start & stop SysEx) will NOT be sent
 (and therefore must be included in the array).
 default value for ArrayContainsBoundaries is set to 'false' for compatibility
 with previous versions of the library.
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendSysEx(unsigned inLength,
                                                    const byte* inArray,
                                                    bool inArrayContainsBoundaries)
{
    const bool writeBeginEndBytes = !inArrayContainsBoundaries;

    if (mTransport.beginTransmission(MidiType::SystemExclusiveStart))
    {
        if (writeBeginEndBytes)
            mTransport.write(MidiType::SystemExclusiveStart);

        for (unsigned i = 0; i < inLength; ++i)
            mTransport.write(inArray[i]);

        if (writeBeginEndBytes)
            mTransport.write(MidiType::SystemExclusiveEnd);

        mTransport.endTransmission();
        updateLastSentTime();
   }

    if (Settings::UseRunningStatus)
        mRunningStatus_TX = InvalidType;

    return *this;
}

/*! \brief Send a Tune Request message.

 When a MIDI unit receives this message,
 it should tune its oscillators (if equipped with any).
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendTuneRequest()
{
    return sendCommon(TuneRequest);
}

/*! \brief Send a MIDI Time Code Quarter Frame.

 \param inTypeNibble      MTC type
 \param inValuesNibble    MTC data
 See MIDI Specification for more information.
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendTimeCodeQuarterFrame(DataByte inTypeNibble,
                                                                            DataByte inValuesNibble)
{
    const byte data = byte((((inTypeNibble & 0x07) << 4) | (inValuesNibble & 0x0f)));
    return sendTimeCodeQuarterFrame(data);
}

/*! \brief Send a MIDI Time Code Quarter Frame.

 See MIDI Specification for more information.
 \param inData  if you want to encode directly the nibbles in your program,
                you can send the byte here.
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendTimeCodeQuarterFrame(DataByte inData)
{
    return sendCommon(TimeCodeQuarterFrame, inData);
}

/*! \brief Send a Song Position Pointer message.
 \param inBeats    The number of beats since the start of the song.
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendSongPosition(unsigned inBeats)
{
    return sendCommon(SongPosition, inBeats);
}

/*! \brief Send a Song Select message */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendSongSelect(DataByte inSongNumber)
{
    return sendCommon(SongSelect, inSongNumber);
}

/*! \brief Send a Common message. Common messages reset the running status.

 \param inType    The available Common types are:
 TimeCodeQuarterFrame, SongPosition, SongSelect and TuneRequest.
 @see MidiType
 \param inData1   The byte that goes with the common message.
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendCommon(MidiType inType, unsigned inData1)
{
    switch (inType)
    {
        case TimeCodeQuarterFrame:
        case SongPosition:
        case SongSelect:
        case TuneRequest:
            break;
        default:
            // Invalid Common marker
            return *this;
    }

    if (mTransport.beginTransmission(inType))
    {
            mTransport.write((byte)inType);
            switch (inType)
            {
            case TimeCodeQuarterFrame:
                mTransport.write(inData1);
                break;
            case SongPosition:
                mTransport.write(inData1 & 0x7f);
                mTransport.write((inData1 >> 7) & 0x7f);
                break;
            case SongSelect:
                mTransport.write(inData1 & 0x7f);
                break;
            case TuneRequest:
                break;
            // LCOV_EXCL_START - Coverage blind spot
            default:
                break;
            // LCOV_EXCL_STOP
        }
        mTransport.endTransmission();
        updateLastSentTime();
    }

    if (Settings::UseRunningStatus)
        mRunningStatus_TX = InvalidType;

    return *this;
}

/*! \brief Send a Real Time (one byte) message.

 \param inType    The available Real Time types are:
 Start, Stop, Continue, Clock, ActiveSensing and SystemReset.
 @see MidiType
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendRealTime(MidiType inType)
{
    // Do not invalidate Running Status for real-time messages
    // as they can be interleaved within any message.

    switch (inType)
    {
        case Clock:
        case Start:
        case Stop:
        case Continue:
        case ActiveSensing:
        case SystemReset:
            if (mTransport.beginTransmission(inType))
            {
                mTransport.write((byte)inType);
                mTransport.endTransmission();
                updateLastSentTime();
            }
            break;
        default:
            // Invalid Real Time marker
            break;
    }

    return *this;
}

/*! \brief Start a Registered Parameter Number frame.
 \param inNumber The 14-bit number of the RPN you want to select.
 \param inChannel The channel on which the message will be sent (1 to 16).
*/
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::beginRpn(unsigned inNumber,
                                                          Channel inChannel)
{
    if (mCurrentRpnNumber != inNumber)
    {
        const byte numMsb = 0x7f & (inNumber >> 7);
        const byte numLsb = 0x7f & inNumber;
        sendControlChange(RPNLSB, numLsb, inChannel);
        sendControlChange(RPNMSB, numMsb, inChannel);
        mCurrentRpnNumber = inNumber;
    }

    return *this;
}

/*! \brief Send a 14-bit value for the currently selected RPN number.
 \param inValue  The 14-bit value of the selected RPN.
 \param inChannel The channel on which the message will be sent (1 to 16).
*/
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendRpnValue(unsigned inValue,
                                                              Channel inChannel)
{;
    const byte valMsb = 0x7f & (inValue >> 7);
    const byte valLsb = 0x7f & inValue;
    sendControlChange(DataEntryMSB, valMsb, inChannel);
    sendControlChange(DataEntryLSB, valLsb, inChannel);

    return *this;
}

/*! \brief Send separate MSB/LSB values for the currently selected RPN number.
 \param inMsb The MSB part of the value to send. Meaning depends on RPN number.
 \param inLsb The LSB part of the value to send. Meaning depends on RPN number.
 \param inChannel The channel on which the message will be sent (1 to 16).
*/
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendRpnValue(byte inMsb,
                                                              byte inLsb,
                                                              Channel inChannel)
{
    sendControlChange(DataEntryMSB, inMsb, inChannel);
    sendControlChange(DataEntryLSB, inLsb, inChannel);

    return *this;
}

/* \brief Increment the value of the currently selected RPN number by the specified amount.
 \param inAmount The amount to add to the currently selected RPN value.
*/
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendRpnIncrement(byte inAmount,
                                                                  Channel inChannel)
{
    sendControlChange(DataIncrement, inAmount, inChannel);

    return *this;
}

/* \brief Decrement the value of the currently selected RPN number by the specified amount.
 \param inAmount The amount to subtract to the currently selected RPN value.
*/
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendRpnDecrement(byte inAmount,
                                                                  Channel inChannel)
{
    sendControlChange(DataDecrement, inAmount, inChannel);

    return *this;
}

/*! \brief Terminate an RPN frame.
This will send a Null Function to deselect the currently selected RPN.
 \param inChannel The channel on which the message will be sent (1 to 16).
*/
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::endRpn(Channel inChannel)
{
    sendControlChange(RPNLSB, 0x7f, inChannel);
    sendControlChange(RPNMSB, 0x7f, inChannel);
    mCurrentRpnNumber = 0xffff;

    return *this;
}



/*! \brief Start a Non-Registered Parameter Number frame.
 \param inNumber The 14-bit number of the NRPN you want to select.
 \param inChannel The channel on which the message will be sent (1 to 16).
*/
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::beginNrpn(unsigned inNumber,
                                                           Channel inChannel)
{
    if (mCurrentNrpnNumber != inNumber)
    {
        const byte numMsb = 0x7f & (inNumber >> 7);
        const byte numLsb = 0x7f & inNumber;
        sendControlChange(NRPNLSB, numLsb, inChannel);
        sendControlChange(NRPNMSB, numMsb, inChannel);
        mCurrentNrpnNumber = inNumber;
    }

    return *this;
}

/*! \brief Send a 14-bit value for the currently selected NRPN number.
 \param inValue  The 14-bit value of the selected NRPN.
 \param inChannel The channel on which the message will be sent (1 to 16).
*/
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendNrpnValue(unsigned inValue,
                                                               Channel inChannel)
{
    const byte valMsb = 0x7f & (inValue >> 7);
    const byte valLsb = 0x7f & inValue;
    sendControlChange(DataEntryMSB, valMsb, inChannel);
    sendControlChange(DataEntryLSB, valLsb, inChannel);

    return *this;
}

/*! \brief Send separate MSB/LSB values for the currently selected NRPN number.
 \param inMsb The MSB part of the value to send. Meaning depends on NRPN number.
 \param inLsb The LSB part of the value to send. Meaning depends on NRPN number.
 \param inChannel The channel on which the message will be sent (1 to 16).
*/
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendNrpnValue(byte inMsb,
                                                               byte inLsb,
                                                               Channel inChannel)
{
    sendControlChange(DataEntryMSB, inMsb, inChannel);
    sendControlChange(DataEntryLSB, inLsb, inChannel);

    return *this;
}

/* \brief Increment the value of the currently selected NRPN number by the specified amount.
 \param inAmount The amount to add to the currently selected NRPN value.
*/
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendNrpnIncrement(byte inAmount,
                                                                   Channel inChannel)
{
    sendControlChange(DataIncrement, inAmount, inChannel);

    return *this;
}

/* \brief Decrement the value of the currently selected NRPN number by the specified amount.
 \param inAmount The amount to subtract to the currently selected NRPN value.
*/
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::sendNrpnDecrement(byte inAmount,
                                                                   Channel inChannel)
{
    sendControlChange(DataDecrement, inAmount, inChannel);

    return *this;
}

/*! \brief Terminate an NRPN frame.
This will send a Null Function to deselect the currently selected NRPN.
 \param inChannel The channel on which the message will be sent (1 to 16).
*/
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::endNrpn(Channel inChannel)
{
    sendControlChange(NRPNLSB, 0x7f, inChannel);
    sendControlChange(NRPNMSB, 0x7f, inChannel);
    mCurrentNrpnNumber = 0xffff;

    return *this;
}

template<class Transport, class Settings, class Platform>
inline void MidiInterface<Transport, Settings, Platform>::updateLastSentTime()
{
    if (Settings::UseSenderActiveSensing && mSenderActiveSensingPeriodicity)
        mLastMessageSentTime = Platform::now();
}

/*! @} */ // End of doc group MIDI Output

// -----------------------------------------------------------------------------

template<class Transport, class Settings, class Platform>
StatusByte MidiInterface<Transport, Settings, Platform>::getStatus(MidiType inType,
                                                          Channel inChannel) const
{
    return StatusByte(((byte)inType | ((inChannel - 1) & 0x0f)));
}

// -----------------------------------------------------------------------------
//                                  Input
// -----------------------------------------------------------------------------

/*! \addtogroup input
 @{
*/

/*! \brief Read messages from the serial port using the main input channel.

 \return True if a valid message has been stored in the structure, false if not.
 A valid message is a message that matches the input channel. \n\n
 If the Thru is enabled and the message matches the filter,
 it is sent back on the MIDI output.
 @see see setInputChannel()
 */
template<class Transport, class Settings, class Platform>
inline bool MidiInterface<Transport, Settings, Platform>::read()
{
    return read(mInputChannel);
}

/*! \brief Read messages on a specified channel.
 */
template<class Transport, class Settings, class Platform>
inline bool MidiInterface<Transport, Settings, Platform>::read(Channel inChannel)
{
    #ifndef RegionActiveSending
    // Active Sensing. This message is intended to be sent
    // repeatedly to tell the receiver that a connection is alive. Use
    // of this message is optional. When initially received, the
    // receiver will expect to receive another Active Sensing
    // message each 300ms (max), and if it does not then it will
    // assume that the connection has been terminated. At
    // termination, the receiver will turn off all voices and return to
    // normal (non- active sensing) operation.
    if (Settings::UseSenderActiveSensing && (mSenderActiveSensingPeriodicity > 0) && (Platform::now() - mLastMessageSentTime) > mSenderActiveSensingPeriodicity)
    {
        sendActiveSensing();
        mLastMessageSentTime = Platform::now();
    }

    if (Settings::UseReceiverActiveSensing && mReceiverActiveSensingActivated && (mLastMessageReceivedTime + ActiveSensingTimeout < Platform::now()))
    {
        mReceiverActiveSensingActivated = false;

        mLastError |= 1UL << ErrorActiveSensingTimeout; // set the ErrorActiveSensingTimeout bit
        if (mErrorCallback)
            mErrorCallback(mLastError);
    }
    #endif

    if (inChannel >= MIDI_CHANNEL_OFF)
        return false; // MIDI Input disabled.

    if (!parse())
        return false;

    #ifndef RegionActiveSending

    if (Settings::UseReceiverActiveSensing && mMessage.type == ActiveSensing)
    {
        // When an ActiveSensing message is received, the time keeping is activated.
        // When a timeout occurs, an error message is send and time keeping ends.
        mReceiverActiveSensingActivated = true;

        // is ErrorActiveSensingTimeout bit in mLastError on
        if (mLastError & (1 << (ErrorActiveSensingTimeout - 1)))
        {
            mLastError &= ~(1UL << ErrorActiveSensingTimeout); // clear the ErrorActiveSensingTimeout bit
            if (mErrorCallback)
                mErrorCallback(mLastError);
        }
    }

    // Keep the time of the last received message, so we can check for the timeout
    if (Settings::UseReceiverActiveSensing && mReceiverActiveSensingActivated)
        mLastMessageReceivedTime = Platform::now();

    #endif

    handleNullVelocityNoteOnAsNoteOff();

    const bool channelMatch = inputFilter(inChannel);
    if (channelMatch)
        launchCallback();

    thruFilter(inChannel);

    return channelMatch;
}

// -----------------------------------------------------------------------------

// Private method: MIDI parser
template<class Transport, class Settings, class Platform>
bool MidiInterface<Transport, Settings, Platform>::parse() {

  if (mTransport.available() == 0)
    return false;

  mLastError &= ~(1UL << ErrorParse);  // Clear ErrorParse bit
  /*Possible Errors:
      > SysEx Stop byte received with no pending SysEx Start.
      > Unsupported Status byte.
      > Data received without a valid Status byte or Running Status.
      > Warning SysEx split warning.
      .... could potentially add an error for when SysEx is aborted due to receiving a new non-realtime status byte. 
  */

  const byte extracted = mTransport.read();

  if (extracted >= 0x80) {
    // Lets try get a valid Status byte. Non-realtime status overides any current Status
    const MidiType pendingType = getTypeFromStatusByte(extracted);
    switch (pendingType) {
      // Realtime
      case Start:
      case Continue:
      case Stop:
      case Clock:
      case Tick:
      case ActiveSensing:
      case SystemReset:
      case TuneRequest:
        // Handle message now
        mMessage.type = pendingType;
        mMessage.channel = 0;
        mMessage.data1 = 0;
        mMessage.data2 = 0;
        mMessage.length = 1;
        mMessage.valid = true;
        return true;
        break;
        // 2 byte messages
      case ProgramChange:
      case AfterTouchChannel:
      case TimeCodeQuarterFrame:
      case SongSelect:
        mPendingMessage[0] = extracted;
        mPendingMessageExpectedLength = 2;
        break;
        // 3 byte messages
      case NoteOn:
      case NoteOff:
      case ControlChange:
      case PitchBend:
      case AfterTouchPoly:
      case SongPosition:
        mPendingMessage[0] = extracted;
        mPendingMessageExpectedLength = 3;
        break;
        // SysEx
      case SystemExclusiveStart:
        mPendingMessage[0] = SystemExclusive;
        mPendingMessageExpectedLength = MidiMessage::sSysExMaxSize;
        mMessage.sysexArray[0] = SystemExclusiveStart;
        mLastError &= ~(1UL << WarningSplitSysEx);  // Reset Warning Split SysEx bit
        break;
      case SystemExclusiveEnd:
        if (mPendingMessage[0] == SystemExclusive) {
          mMessage.sysexArray[mPendingMessageIndex++] = SystemExclusiveEnd;  // Post Inc pending index here for correct lenght data
          mMessage.type = SystemExclusive;
          mMessage.data1 = mPendingMessageIndex & 0xff;      // LSB
          mMessage.data2 = byte(mPendingMessageIndex >> 8);  // MSB
          mMessage.channel = 0;
          mMessage.length = mPendingMessageIndex;
          if (mMessage.sysexArray[0] == SystemExclusiveEnd) {
            // This is the last chunk of a split SysEx message, and is NOT a valid SysEx message (it starts with 0xF7)
            mMessage.valid = false;  // SysEx message is split so this not technically valid
            launchCallback();        // Lets notify callback to deal with this
            resetInput();            // Restart message
            return false;
          } else {
            // We are in a valid SysEx message that hasn't overrun (starts with 0xF0) so lets complete it
            mMessage.valid = true;
            resetInput();  // Restart message
            return true;
          }
        } else {                            // Looks like a SysEx End without a Sysex Start
          mLastError |= 1UL << ErrorParse;  // Error: SysEx Stop byte received with no pending SysEx Start.
          if (mErrorCallback)
            mErrorCallback(mLastError);
          resetInput();  // Restart message
          return false;
        }
        break;
      // Unsupported
      default:
        mPendingMessage[0] = InvalidType;
        mLastError |= 1UL << ErrorParse;  // Error: Unsupported Status byte.
        if (mErrorCallback)
          mErrorCallback(mLastError);
        resetInput();  // Restart message
        return false;
        break;
    }
    mPendingMessageIndex = 1;         // If we are here, we have a valid Status! Lets try get some Data for it....
    mRunningStatus_RX = InvalidType;  // Lets also reset Running Status until we have a complete message
    return (Settings::Use1ByteParsing) ? false : parse();

  } else {
    // Lets get some data... First off.. check for Status Byte, or use Running Status
    if (mPendingMessageIndex == 0) {
      if (mRunningStatus_RX) {
        // Yay! We have Running Status
        mPendingMessage[0] = mRunningStatus_RX;
        mPendingMessageIndex = 1;
      } else {
        // ooops.... No Status Byte... No Running Status... lets ignore this data
        mLastError |= 1UL << ErrorParse;  // Error: Data received without a valid Status byte or Running Status.
        if (mErrorCallback)
          mErrorCallback(mLastError);
        return false;
      }
    }

    // Status or Running Status is good so add extracted data byte to pending message
    if (mPendingMessage[0] == SystemExclusive)
      mMessage.sysexArray[mPendingMessageIndex] = extracted;
    else
      mPendingMessage[mPendingMessageIndex] = extracted;

    // Now we are going to check if we have reached the end of the message
    if (mPendingMessageIndex >= (mPendingMessageExpectedLength - 1)) {
      // SysEx larger than the allocated buffer size,
      // Split SysEx like so:
      //   first:  0xF0 .... 0xF0
      //   middle: 0xF7 .... 0xF0
      //   last:   0xF7 .... 0xF7
      // ***** If the buffer has overrun, this SysEx message can now no longer be considered a valid Midi message and must be dealt with via callbacks only! ****
      if (mPendingMessage[0] == SystemExclusive) {
        // Warn at start of SysEx split
        if (mMessage.sysexArray[0] == SystemExclusiveStart) {
          mLastError |= 1UL << WarningSplitSysEx;  // We have this error already defined so may as well use it
          if (mErrorCallback)
            mErrorCallback(mLastError);
        }
        auto lastByte = mMessage.sysexArray[Settings::SysExMaxSize - 1];
        mMessage.sysexArray[Settings::SysExMaxSize - 1] = SystemExclusiveStart;
        mMessage.type = SystemExclusive;

        // Get length
        mMessage.data1 = Settings::SysExMaxSize & 0xff;      // LSB
        mMessage.data2 = byte(Settings::SysExMaxSize >> 8);  // MSB
        mMessage.channel = 0;
        mMessage.length = Settings::SysExMaxSize;
        mMessage.valid = false;  // SysEx message is split so this is not technically valid

        // Notify callback to deal with the SysEx data chunk
        launchCallback();

        // Prep next SysEx data chunk to start with 0xF7
        mMessage.sysexArray[0] = SystemExclusiveEnd;
        mMessage.sysexArray[1] = lastByte;
        mPendingMessageIndex = 2;
        // SysEx buffer has overrun so parse() will no longer return true, and will need to be dealt with via callbacks only
        return false;
      }

      // Pending message is complete so lets save it
      mMessage.type = getTypeFromStatusByte(mPendingMessage[0]);

      if (isChannelMessage(mMessage.type)) {
        mMessage.channel = getChannelFromStatusByte(mPendingMessage[0]);
        // Message will be completed soon so lets update RunningStatus now as this is obviously a valid Channel Message.
        mRunningStatus_RX = mPendingMessage[0];
      } else
        mMessage.channel = 0;

      mMessage.data1 = mPendingMessage[1];
      // Save data2 only if applicable
      mMessage.data2 = mPendingMessageExpectedLength == 3 ? mPendingMessage[2] : 0;
      mMessage.length = mPendingMessageExpectedLength;
      mMessage.valid = true;

      // Reset index for next message
      mPendingMessageIndex = 0;
      //mPendingMessageExpectedLength = 0; // <-- No need to reset this as its valid still for Running Status, or will be updated on next Status byte received.

      return true;
    } else {
      // We need more data...
      mPendingMessageIndex++;

      return (Settings::Use1ByteParsing) ? false : parse();
    }
  }
}

// Private method, see midi_Settings.h for documentation
template<class Transport, class Settings, class Platform>
inline void MidiInterface<Transport, Settings, Platform>::handleNullVelocityNoteOnAsNoteOff()
{
    if (Settings::HandleNullVelocityNoteOnAsNoteOff &&
        getType() == NoteOn && getData2() == 0)
    {
        mMessage.type = NoteOff;
    }
}

// Private method: check if the received message is on the listened channel
template<class Transport, class Settings, class Platform>
inline bool MidiInterface<Transport, Settings, Platform>::inputFilter(Channel inChannel)
{
    // This method handles recognition of channel
    // (to know if the message is destinated to the Arduino)

    // First, check if the received message is Channel
    if (mMessage.type >= NoteOff && mMessage.type <= PitchBend)
    {
        // Then we need to know if we listen to it
        if ((mMessage.channel == inChannel) ||
            (inChannel == MIDI_CHANNEL_OMNI))
        {
            return true;
        }
        else
        {
            // We don't listen to this channel
            return false;
        }
    }
    else
    {
        // System messages are always received
        return true;
    }
}

// Private method: reset input attributes
template<class Transport, class Settings, class Platform>
inline void MidiInterface<Transport, Settings, Platform>::resetInput()
{
    mPendingMessageIndex = 0;
    mPendingMessageExpectedLength = 0;
    mRunningStatus_RX = InvalidType;
}

// -----------------------------------------------------------------------------

/*! \brief Get the last received message's type

 Returns an enumerated type. @see MidiType
 */
template<class Transport, class Settings, class Platform>
inline MidiType MidiInterface<Transport, Settings, Platform>::getType() const
{
    return mMessage.type;
}

/*! \brief Get the channel of the message stored in the structure.

 \return Channel range is 1 to 16.
 For non-channel messages, this will return 0.
 */
template<class Transport, class Settings, class Platform>
inline Channel MidiInterface<Transport, Settings, Platform>::getChannel() const
{
    return mMessage.channel;
}

/*! \brief Get the first data byte of the last received message. */
template<class Transport, class Settings, class Platform>
inline DataByte MidiInterface<Transport, Settings, Platform>::getData1() const
{
    return mMessage.data1;
}

/*! \brief Get the second data byte of the last received message. */
template<class Transport, class Settings, class Platform>
inline DataByte MidiInterface<Transport, Settings, Platform>::getData2() const
{
    return mMessage.data2;
}

/*! \brief Get the System Exclusive byte array.

 @see getSysExArrayLength to get the array's length in bytes.
 */
template<class Transport, class Settings, class Platform>
inline const byte* MidiInterface<Transport, Settings, Platform>::getSysExArray() const
{
    return mMessage.sysexArray;
}

/*! \brief Get the length of the System Exclusive array.

 It is coded using data1 as LSB and data2 as MSB.
 \return The array's length, in bytes.
 */
template<class Transport, class Settings, class Platform>
inline unsigned MidiInterface<Transport, Settings, Platform>::getSysExArrayLength() const
{
    return mMessage.getSysExSize();
}

/*! \brief Check if a valid message is stored in the structure. */
template<class Transport, class Settings, class Platform>
inline bool MidiInterface<Transport, Settings, Platform>::check() const
{
    return mMessage.valid;
}

// -----------------------------------------------------------------------------

template<class Transport, class Settings, class Platform>
inline Channel MidiInterface<Transport, Settings, Platform>::getInputChannel() const
{
    return mInputChannel;
}

/*! \brief Set the value for the input MIDI channel
 \param inChannel the channel value. Valid values are 1 to 16, MIDI_CHANNEL_OMNI
 if you want to listen to all channels, and MIDI_CHANNEL_OFF to disable input.
 */
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::setInputChannel(Channel inChannel)
{
    mInputChannel = inChannel;

    return *this;
}

// -----------------------------------------------------------------------------

/*! \brief Extract an enumerated MIDI type from a status byte.

 This is a utility static method, used internally,
 made public so you can handle MidiTypes more easily.
 */
template<class Transport, class Settings, class Platform>
MidiType MidiInterface<Transport, Settings, Platform>::getTypeFromStatusByte(byte inStatus)
{
    if ((inStatus  < 0x80) ||
        (inStatus == Undefined_F4) ||
        (inStatus == Undefined_F5) ||
        (inStatus == Undefined_FD))
        return InvalidType; // Data bytes and undefined.

    if (inStatus < 0xf0)
        // Channel message, remove channel nibble.
        return MidiType(inStatus & 0xf0);

    return MidiType(inStatus);
}

/*! \brief Returns channel in the range 1-16
 */
template<class Transport, class Settings, class Platform>
inline Channel MidiInterface<Transport, Settings, Platform>::getChannelFromStatusByte(byte inStatus)
{
    return Channel((inStatus & 0x0f) + 1);
}

template<class Transport, class Settings, class Platform>
bool MidiInterface<Transport, Settings, Platform>::isChannelMessage(MidiType inType)
{
    return (inType == NoteOff           ||
            inType == NoteOn            ||
            inType == ControlChange     ||
            inType == AfterTouchPoly    ||
            inType == AfterTouchChannel ||
            inType == PitchBend         ||
            inType == ProgramChange);
}

// -----------------------------------------------------------------------------

/*! \brief Detach an external function from the given type.

 Use this method to cancel the effects of setHandle********.
 \param inType        The type of message to unbind.
 When a message of this type is received, no function will be called.
 */
template<class Transport, class Settings, class Platform>
MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::disconnectCallbackFromType(MidiType inType)
{
    switch (inType)
    {
        case NoteOff:               mNoteOffCallback                = nullptr; break;
        case NoteOn:                mNoteOnCallback                 = nullptr; break;
        case AfterTouchPoly:        mAfterTouchPolyCallback         = nullptr; break;
        case ControlChange:         mControlChangeCallback          = nullptr; break;
        case ProgramChange:         mProgramChangeCallback          = nullptr; break;
        case AfterTouchChannel:     mAfterTouchChannelCallback      = nullptr; break;
        case PitchBend:             mPitchBendCallback              = nullptr; break;
        case SystemExclusive:       mSystemExclusiveCallback        = nullptr; break;
        case TimeCodeQuarterFrame:  mTimeCodeQuarterFrameCallback   = nullptr; break;
        case SongPosition:          mSongPositionCallback           = nullptr; break;
        case SongSelect:            mSongSelectCallback             = nullptr; break;
        case TuneRequest:           mTuneRequestCallback            = nullptr; break;
        case Clock:                 mClockCallback                  = nullptr; break;
        case Start:                 mStartCallback                  = nullptr; break;
        case Tick:                  mTickCallback                   = nullptr; break;
        case Continue:              mContinueCallback               = nullptr; break;
        case Stop:                  mStopCallback                   = nullptr; break;
        case ActiveSensing:         mActiveSensingCallback          = nullptr; break;
        case SystemReset:           mSystemResetCallback            = nullptr; break;
        default:
            break;
    }

    return *this;
}

/*! @} */ // End of doc group MIDI Callbacks

// Private - launch callback function based on received type.
template<class Transport, class Settings, class Platform>
void MidiInterface<Transport, Settings, Platform>::launchCallback()
{
    if (mMessageCallback != 0) mMessageCallback(mMessage);

    // The order is mixed to allow frequent messages to trigger their callback faster.
    switch (mMessage.type)
    {
            // Notes
        case NoteOff:               if (mNoteOffCallback != nullptr)               mNoteOffCallback(mMessage.channel, mMessage.data1, mMessage.data2);   break;
        case NoteOn:                if (mNoteOnCallback != nullptr)                mNoteOnCallback(mMessage.channel, mMessage.data1, mMessage.data2);    break;

            // Real-time messages
        case Clock:                 if (mClockCallback != nullptr)                 mClockCallback();           break;
        case Start:                 if (mStartCallback != nullptr)                 mStartCallback();           break;
        case Tick:                  if (mTickCallback != nullptr)                  mTickCallback();            break;
        case Continue:              if (mContinueCallback != nullptr)              mContinueCallback();        break;
        case Stop:                  if (mStopCallback != nullptr)                  mStopCallback();            break;
        case ActiveSensing:         if (mActiveSensingCallback != nullptr)         mActiveSensingCallback();   break;

            // Continuous controllers
        case ControlChange:         if (mControlChangeCallback != nullptr)         mControlChangeCallback(mMessage.channel, mMessage.data1, mMessage.data2);    break;
        case PitchBend:             if (mPitchBendCallback != nullptr)             mPitchBendCallback(mMessage.channel, (int)((mMessage.data1 & 0x7f) | ((mMessage.data2 & 0x7f) << 7)) + MIDI_PITCHBEND_MIN); break;
        case AfterTouchPoly:        if (mAfterTouchPolyCallback != nullptr)        mAfterTouchPolyCallback(mMessage.channel, mMessage.data1, mMessage.data2);    break;
        case AfterTouchChannel:     if (mAfterTouchChannelCallback != nullptr)     mAfterTouchChannelCallback(mMessage.channel, mMessage.data1);    break;

        case ProgramChange:         if (mProgramChangeCallback != nullptr)         mProgramChangeCallback(mMessage.channel, mMessage.data1);    break;
        case SystemExclusive:       if (mSystemExclusiveCallback != nullptr)       mSystemExclusiveCallback(mMessage.sysexArray, mMessage.getSysExSize());    break;

            // Occasional messages
        case TimeCodeQuarterFrame:  if (mTimeCodeQuarterFrameCallback != nullptr)  mTimeCodeQuarterFrameCallback(mMessage.data1);    break;
        case SongPosition:          if (mSongPositionCallback != nullptr)          mSongPositionCallback(unsigned((mMessage.data1 & 0x7f) | ((mMessage.data2 & 0x7f) << 7)));    break;
        case SongSelect:            if (mSongSelectCallback != nullptr)            mSongSelectCallback(mMessage.data1);    break;
        case TuneRequest:           if (mTuneRequestCallback != nullptr)           mTuneRequestCallback();    break;

        case SystemReset:           if (mSystemResetCallback != nullptr)           mSystemResetCallback();    break;

        // LCOV_EXCL_START - Unreacheable code, but prevents unhandled case warning.
        case InvalidType:
        default:
            break;
        // LCOV_EXCL_STOP
    }
}

/*! @} */ // End of doc group MIDI Input

// -----------------------------------------------------------------------------
//                                  Thru
// -----------------------------------------------------------------------------

/*! \addtogroup thru
 @{
 */

/*! \brief Set the filter for thru mirroring
 \param inThruFilterMode a filter mode

 @see Thru::Mode
 */
template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::setThruFilterMode(Thru::Mode inThruFilterMode)
{
    mThruFilterMode = inThruFilterMode;
    mThruActivated  = mThruFilterMode != Thru::Off;

    return *this;
}

template<class Transport, class Settings, class Platform>
inline Thru::Mode MidiInterface<Transport, Settings, Platform>::getFilterMode() const
{
    return mThruFilterMode;
}

template<class Transport, class Settings, class Platform>
inline bool MidiInterface<Transport, Settings, Platform>::getThruState() const
{
    return mThruActivated;
}

template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::turnThruOn(Thru::Mode inThruFilterMode)
{
    mThruActivated = true;
    mThruFilterMode = inThruFilterMode;

    return *this;
}

template<class Transport, class Settings, class Platform>
inline MidiInterface<Transport, Settings, Platform>& MidiInterface<Transport, Settings, Platform>::turnThruOff()
{
    mThruActivated = false;
    mThruFilterMode = Thru::Off;

    return *this;
}


/*! @} */ // End of doc group MIDI Thru

// This method is called upon reception of a message
// and takes care of Thru filtering and sending.
// - All system messages (System Exclusive, Common and Real Time) are passed
//   to output unless filter is set to Off.
// - Channel messages are passed to the output whether their channel
//   is matching the input channel and the filter setting
template<class Transport, class Settings, class Platform>
void MidiInterface<Transport, Settings, Platform>::thruFilter(Channel inChannel)
{
    // If the feature is disabled, don't do anything.
    if (!mThruActivated || (mThruFilterMode == Thru::Off))
        return;

    // First, check if the received message is Channel
    if (mMessage.type >= NoteOff && mMessage.type <= PitchBend)
    {
        const bool filter_condition = ((mMessage.channel == inChannel) ||
                                       (inChannel == MIDI_CHANNEL_OMNI));

        // Now let's pass it to the output
        switch (mThruFilterMode)
        {
            case Thru::Full:
                send(mMessage.type,
                     mMessage.data1,
                     mMessage.data2,
                     mMessage.channel);
                break;

            case Thru::SameChannel:
                if (filter_condition)
                {
                    send(mMessage.type,
                         mMessage.data1,
                         mMessage.data2,
                         mMessage.channel);
                }
                break;

            case Thru::DifferentChannel:
                if (!filter_condition)
                {
                    send(mMessage.type,
                         mMessage.data1,
                         mMessage.data2,
                         mMessage.channel);
                }
                break;

            default:
                break;
        }
    }
    else
    {
        // Send the message to the output
        switch (mMessage.type)
        {
                // Real Time and 1 byte
            case Clock:
            case Start:
            case Stop:
            case Continue:
            case ActiveSensing:
            case SystemReset:
            case TuneRequest:
                sendRealTime(mMessage.type);
                break;

            case SystemExclusive:
                // Send SysEx (0xf0 and 0xf7 are included in the buffer)
                sendSysEx(getSysExArrayLength(), getSysExArray(), true);
                break;

            case SongSelect:
                sendSongSelect(mMessage.data1);
                break;

            case SongPosition:
                sendSongPosition(mMessage.data1 | ((unsigned)mMessage.data2 << 7));
                break;

            case TimeCodeQuarterFrame:
                sendTimeCodeQuarterFrame(mMessage.data1,mMessage.data2);
                break;

            default:
                break; // LCOV_EXCL_LINE - Unreacheable code, but prevents unhandled case warning.
        }
    }
}

END_MIDI_NAMESPACE
