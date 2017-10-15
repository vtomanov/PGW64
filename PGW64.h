/**
   USE OF THIS SOFTWARE IS GOVERNED BY THE TERMS AND CONDITIONS
   OF THE LICENSE STATEMENT AND LIMITED WARRANTY FURNISHED WITH
   THE PRODUCT.
   <p/>
   IN PARTICULAR, YOU WILL INDEMNIFY AND HOLD ITS AUTHOR, ITS
   RELATED ENTITIES AND ITS SUPPLIERS, HARMLESS FROM AND AGAINST ANY
   CLAIMS OR LIABILITIES ARISING OUT OF THE USE, REPRODUCTION, OR
   DISTRIBUTION OF YOUR PROGRAMS, INCLUDING ANY CLAIMS OR LIABILITIES
   ARISING OUT OF OR RESULTING FROM THE USE, MODIFICATION, OR
   DISTRIBUTION OF PROGRAMS OR FILES CREATED FROM, BASED ON, AND/OR
   DERIVED FROM THIS SOURCE CODE FILE.
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Background communication library for 433mhtz RF implementation
//

#include "TimerOne.h"

///////////////////////////////////////////////////////////////////////////////////
// LOG constants
//#define LOG64_ENABLED
//#define EEPROM_LOG64_ENABLED

//#include <Log64.h>

//////////////////////////////////////////////////////////////////////////////////////
// ALL AVAILABLE FUNCTIONS ( .h )

#define PGW64_MAX_BUF_SIZE 30

inline void PGW64_INIT(uint8_t TX_PIN, uint8_t RX_PIN, uint8_t TX_POWER_PIN);

// While sending the library will ignore all received data
// Return false if previouse packet is still not finished sending
inline bool PGW64_SEND(const uint8_t BUF[], const uint8_t SIZE);

// Return true if packet is received and transfered in the buffer
inline bool PGW64_RECEIVE(uint8_t BUF[], uint8_t & SIZE);

// Return true if previouse packet is still not finished sending
inline bool PGW64_IS_SENDING();

//////////////////////////////////////////////////////////////////////////////////////
// Timer related - helpers for simulated multithreading - as the timer1 is used by the library

// Current value of timer - in millis from start
inline uint64_t PGW64_TIMER_GET();

// Will return true only if the function is ready for execute
inline bool PGW64_DO_EXECUTE(uint64_t loopCounter, uint64_t lastExecute, uint32_t timeout);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATIONS

//FINAL ONE !

//////////////////////////////////////////////////////////////////////////////////////
// Prevent the compiler from  optimising the code
//#pragma GCC push_options
//#pragma GCC optimize ("O0")

#define PGW64_RETRANSMIT_COUNT 14
#define PGW64_MAX_INTERNAL_BUF_SIZE ((PGW64_MAX_BUF_SIZE * 2) + 4)

uint64_t PGW64_TIMER_10_millis;
uint8_t PGW64_COUNTER;

uint8_t PGW64_TX_PIN;
uint8_t PGW64_RX_PIN;
uint8_t PGW64_TX_POWER_PIN;
uint8_t PGW64_SEND_BUF[PGW64_MAX_INTERNAL_BUF_SIZE];
uint8_t PGW64_SEND_SIZE;
uint8_t PGW64_SEND_INDEX;
uint8_t PGW64_SEND_BIT;

uint8_t PGW64_RECV_BUF[PGW64_MAX_INTERNAL_BUF_SIZE];
uint8_t PGW64_RECV_SIZE;
uint8_t PGW64_RECV_INDEX;
uint8_t PGW64_RECV_BYTE;
uint8_t PGW64_RECV_BIT;
uint8_t PGW64_RECV_ZERROS;
uint8_t PGW64_RECV_ONES;
bool    PGW64_RECV_ESC;
uint8_t PGW64_PAT[PGW64_RETRANSMIT_COUNT];

// choose a number that has first bit set to 1 or the logic will not work
#define PGW64_DELIMITER 85
#define PGW64_ESCAPE 204


void PGW64_CALLBACK_10_MSEC(); // the timer callback 10 times every millisecond

#define PGW64_CHECK_BIT(var,pos) ((var) & (1<<(pos)))

inline void PGW64_INIT(uint8_t TX_PIN, uint8_t RX_PIN, uint8_t TX_POWER_PIN)
{
  PGW64_SEND_SIZE = 0;
  PGW64_SEND_INDEX = 0;
  PGW64_SEND_BIT = 0;

  PGW64_RECV_SIZE = 0;
  PGW64_RECV_INDEX = 0;
  PGW64_RECV_BYTE = 0;
  PGW64_RECV_BIT = 0;
  PGW64_RECV_ZERROS = 0;
  PGW64_RECV_ONES = 0;
  PGW64_RECV_ESC = false;

  PGW64_COUNTER = 0;

  PGW64_TX_PIN = TX_PIN;
  PGW64_RX_PIN = RX_PIN;
  PGW64_TX_POWER_PIN = TX_POWER_PIN;

  pinMode(PGW64_TX_PIN, OUTPUT);
  pinMode(PGW64_TX_POWER_PIN, OUTPUT);
  pinMode(PGW64_RX_PIN, INPUT);

  digitalWrite(PGW64_TX_POWER_PIN, LOW);

  PGW64_TIMER_10_millis = 0;

  // 100 is 10 times per millisecond
  // 10000 is 10 msec
  Timer1.initialize(100);                      // initialise timer1, and set a 10 times per millisecond period
  Timer1.attachInterrupt(PGW64_CALLBACK_10_MSEC);  // attaches callback() as a timer overflow interrupt
}

uint8_t cc = 0;

inline void PGW64_ENCODE(uint8_t DEST[], const uint8_t SOURCE[], uint8_t & SIZE)
{
  uint8_t j = 0;
  uint8_t s = 0;
  DEST[j++] = PGW64_DELIMITER;
  DEST[j++] = PGW64_DELIMITER;
  for (uint8_t i = 0; i < SIZE; i++)
  {
    s += SOURCE[i];
  }
  DEST[j++] = s;
  for (uint8_t i = 0; i < SIZE; i++)
  {
    switch (SOURCE[i])
    {
      case PGW64_DELIMITER :
        {
          DEST[j++] = PGW64_ESCAPE;
          DEST[j++] = PGW64_DELIMITER;
        }; break;
      case PGW64_ESCAPE :
        {
          DEST[j++] = PGW64_ESCAPE;
          DEST[j++] = PGW64_ESCAPE;
        }; break;
      default :
        {
          DEST[j++] = SOURCE[i];
        }
    }
  }
  DEST[j++] = (((cc++) % 2 ) == 0) ? PGW64_DELIMITER : (PGW64_DELIMITER + 1);
  SIZE = j;
}

inline void PGW64_DECODE(uint8_t DEST[], const uint8_t SOURCE[], uint8_t & SIZE)
{
  SIZE -= 3;
  memcpy(DEST, & SOURCE[2], SIZE);
}


inline bool PGW64_SEND(const uint8_t BUF[], const uint8_t SIZE)
{
  if (PGW64_SEND_SIZE > 0)
  {
    return false;
  }

  digitalWrite(PGW64_TX_POWER_PIN, HIGH);

  uint8_t tmp_size = SIZE;
  PGW64_ENCODE(PGW64_SEND_BUF, BUF, tmp_size);

  //  {
  //    for (uint8_t i = 0; i < tmp_size; i++)
  //    {
  //      LOG64_SET(PGW64_SEND_BUF[i]);
  //    }
  //    LOG64_NEW_LINE;
  //  }

  PGW64_SEND_SIZE = tmp_size;

  return true;
}

inline bool PGW64_IS_SENDING()
{
  return (PGW64_SEND_SIZE > 0);
}


inline bool PGW64_RECEIVE(uint8_t BUF[], uint8_t & SIZE)
{
  if (PGW64_RECV_SIZE == 0)
  {
    // no data
    return false;
  }

  PGW64_DECODE(BUF, PGW64_RECV_BUF, PGW64_RECV_SIZE);
  SIZE = PGW64_RECV_SIZE;
  // checksum
  bool ret = false;
  uint8_t s = 0;
  for (uint8_t i = 0; i < SIZE; i++)
  {
    s += BUF[i];
  }

  if (s == PGW64_RECV_BUF[1])
  {
    ret = true;
  }

  // reset the receiver
  PGW64_RECV_INDEX = 0;
  PGW64_RECV_BYTE = 0;
  PGW64_RECV_BIT = 0;
  PGW64_RECV_ZERROS = 0;
  PGW64_RECV_ONES = 0;
  PGW64_RECV_ESC = false;
  PGW64_RECV_SIZE = 0;


  return ret;
}


void PGW64_CALLBACK_10_MSEC()
{

  PGW64_TIMER_10_millis++;

  if (PGW64_SEND_SIZE > 0)
  {
    // sending mode
    if (PGW64_COUNTER == PGW64_RETRANSMIT_COUNT)
    {
      PGW64_COUNTER = 0;

      if (PGW64_SEND_INDEX == PGW64_SEND_SIZE)
      {

        //        {
        //          LOG64_SET("PACKET FULLY SENT");
        //          LOG64_NEW_LINE;
        //        }
        PGW64_SEND_INDEX = 0;
        PGW64_SEND_SIZE = 0;

        digitalWrite(PGW64_TX_PIN, LOW);

        digitalWrite(PGW64_TX_POWER_PIN, LOW);


        // reset the receiver

        PGW64_RECV_INDEX = 0;
        PGW64_RECV_BYTE = 0;
        PGW64_RECV_BIT = 0;
        PGW64_RECV_ZERROS = 0;
        PGW64_RECV_ONES = 0;
        PGW64_RECV_ESC = false;
        PGW64_RECV_SIZE = 0;
      }
      else
      {
        //        {
        //          LOG64_SET("--------------------------------");
        //          LOG64_NEW_LINE;
        //          LOG64_SET("SENDING BYTE NO:");
        //          LOG64_SET(PGW64_SEND_INDEX);
        //          LOG64_SET("SENDING BIT NO:");
        //          LOG64_SET(PGW64_SEND_BIT);
        //          LOG64_SET("VAL: ");
        //        }
        if (PGW64_CHECK_BIT(PGW64_SEND_BUF[PGW64_SEND_INDEX], PGW64_SEND_BIT++))
        {
          digitalWrite(PGW64_TX_PIN, HIGH);
          //          LOG64_SET(1);
        }
        else
        {
          digitalWrite(PGW64_TX_PIN, LOW);
          //          LOG64_SET(0);
        }

        //        LOG64_NEW_LINE;

        if (PGW64_SEND_BIT > 7)
        {
          PGW64_SEND_BIT = 0;
          PGW64_SEND_INDEX++;
        }
      }
    }
    PGW64_COUNTER++;
  }
  else
  {
    // we are in receiving mode
    // check if the buffer is ready to receive or ignore
    if (PGW64_RECV_SIZE == 0)
    {
      if (PGW64_COUNTER == PGW64_RETRANSMIT_COUNT)
      {
        PGW64_COUNTER = 0;
        //        {
        //                    LOG64_SET("RECEIVE INDEX :");
        //                    LOG64_SET(PGW64_RECV_INDEX);
        //                    LOG64_SET("BIT 0:");
        //                    LOG64_SET(PGW64_RECV_ZERROS);
        //                    LOG64_SET(" 1:");
        //                    LOG64_SET(PGW64_RECV_ONES);
        //                    LOG64_NEW_LINE;
        //                    for (int i = 0; i < PGW64_RETRANSMIT_COUNT; i++)
        //                    {
        //                      LOG64_SET(PGW64_PAT[i]);
        //                    }
        //                    LOG64_NEW_LINE;
        //        }

        if (PGW64_RECV_ZERROS <= PGW64_RECV_ONES)
        {
          // set to  1
          PGW64_RECV_BYTE |= 1 << PGW64_RECV_BIT;
        }
        // no need to set to 0 as by init all is 0 // number &= ~(1 << x);
        PGW64_RECV_ZERROS = 0;
        PGW64_RECV_ONES = 0;

        //analize the pettern and try to sync

        {
            uint8_t pat_change = 0;
            uint8_t pat_val = PGW64_PAT[0];
            for (uint8_t i = 1, tmp_pat_count = 1, pat_count = 1; i < (PGW64_RETRANSMIT_COUNT - 2); i++, tmp_pat_count++)
            {
              if ((PGW64_PAT[i - 1] != PGW64_PAT[i]) && (PGW64_PAT[i] == PGW64_PAT[i + 1]) && (PGW64_PAT[i] == PGW64_PAT[i + 2]))
              {
                if ((tmp_pat_count >= pat_count) && (tmp_pat_count > 1))
                {
                  pat_count = tmp_pat_count;
                  pat_change = i;
                  tmp_pat_count = 0;
                  pat_val = PGW64_PAT[i];
                }
              }
            }
            // adjust sync and fill preliminary bits
            if (pat_change < (PGW64_RETRANSMIT_COUNT / 2))
            {
              // we have read from previouse bit - need to skip in next
              PGW64_COUNTER = 0 - pat_change;
            }
            else
            {
              // adjust for first bit from the packet resync
              if ((PGW64_RECV_BIT == 0) && (PGW64_RECV_INDEX == 0))
              {
                // set the first bit ( in this case the byte has only one bit set) to the val
                PGW64_RECV_BYTE = pat_val;
                // we have read from previouse bit - need to skip in next
                PGW64_COUNTER = 0 - pat_change;
              }
              else
              {
                // we have read from next bit need to move it to next
                // no need to handle pat_change == 0 - it is handled in upper section // ((pat_change == 0) ? (byte)0 : (byte)(PGW64_RETRANSMIT_COUNT - pat_change));
                PGW64_COUNTER = PGW64_RETRANSMIT_COUNT - pat_change;
                memset(PGW64_PAT, pat_val, PGW64_COUNTER);
                if (pat_val == 0)
                {
                  PGW64_RECV_ZERROS = PGW64_COUNTER;
                }
                else
                {
                  PGW64_RECV_ONES = PGW64_COUNTER;
                }
              }
            }
          }

          if ((PGW64_RECV_BIT == 0) && (PGW64_RECV_INDEX == 0) && (PGW64_RECV_BYTE == 0))
          {
            // we have not started a packet - out packet always starts with bit with 1
            // do not ivrease the bit position
          }
          else
          {
            PGW64_RECV_BIT++;
          }

          if (PGW64_RECV_BIT == 8)
          {
            //          {
            //            LOG64_SET(PGW64_RECV_BYTE);
            //            LOG64_NEW_LINE;
            //          }

            PGW64_RECV_BIT = 0;

            if ((PGW64_RECV_INDEX > 0) || ((PGW64_RECV_INDEX == 0) && (PGW64_RECV_BYTE == PGW64_DELIMITER)))
            {

              if ((PGW64_RECV_INDEX == 1) && (PGW64_RECV_BYTE == PGW64_DELIMITER) && (!PGW64_RECV_ESC))
              {
                // start packet again - skip
                PGW64_RECV_INDEX--;
              }

              if ((!PGW64_RECV_ESC) && (PGW64_RECV_BYTE == PGW64_ESCAPE))
              {
                PGW64_RECV_ESC = true;

                // clear the byte and continue to assemble
                PGW64_RECV_BYTE = 0;
              }
              else
              {

                PGW64_RECV_BUF[PGW64_RECV_INDEX++] = PGW64_RECV_BYTE;

                // check for end
                if ((!PGW64_RECV_ESC) && ((PGW64_RECV_INDEX > 1) && (PGW64_RECV_BYTE == PGW64_DELIMITER)))
                {
                  //                {
                  //                  LOG64_SET("PACKET RECEIVED:");
                  //                  LOG64_SET(PGW64_RECV_INDEX);
                  //                  LOG64_NEW_LINE;
                  //                }
                  // packed finished
                  PGW64_RECV_SIZE = PGW64_RECV_INDEX;
                  // no need to clear anything else when read or reset will be cleared
                }
                else
                {
                  PGW64_RECV_ESC = false;

                  // check if something wrong and need reset
                  if (PGW64_RECV_INDEX == PGW64_MAX_INTERNAL_BUF_SIZE)
                  {
                    // reset receiver
                    PGW64_RECV_INDEX = 0;
                    PGW64_RECV_BYTE = 0;
                    PGW64_RECV_BIT = 0;
                    PGW64_RECV_ZERROS = 0;
                    PGW64_RECV_ONES = 0;
                    PGW64_RECV_ESC = false;
                    PGW64_RECV_SIZE = 0;
                  }
                  else
                  {
                    // clear the byte and continue to assemble
                    PGW64_RECV_BYTE = 0;
                  }
                }
              }
            }
            else
            {
              // clear the byte and continue to assemble
              PGW64_RECV_BYTE = 0;
            }
          }
        }

        // assemble the bit
        if (PGW64_COUNTER < (255 - PGW64_RETRANSMIT_COUNT))
        {
          switch (digitalRead(PGW64_RX_PIN))
          {
            case 0:
              {
                // ATAD // inverted case as ASK
                PGW64_PAT[PGW64_COUNTER] = 0;
                PGW64_RECV_ZERROS++;
              }; break;
            case 1:
              {
                // ATAD // inverted case as ASK
                PGW64_RECV_ONES++;
                PGW64_PAT[PGW64_COUNTER] = 1;
              }; break;
          }
        }

        PGW64_COUNTER++;
      }
    }
  }

  inline uint64_t PGW64_TIMER_GET()
  {
    return (PGW64_TIMER_10_millis / 10) ;
  }

  inline bool PGW64_DO_EXECUTE(uint64_t loopCounter, uint64_t lastExecute, uint32_t timeout)
  {
    if ((lastExecute + ((uint64_t)timeout)) < loopCounter)
    {
      return true;
    }

    return false;
  }

  //////////////////////////////////////////////////////////////////////////////////////
  //Restore the prevention of the compiler from  optimising the code
  //#pragma GCC pop_options
  
  
