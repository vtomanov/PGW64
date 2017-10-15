# PGW64
Pretty Good Wire (transport library aiming to transport max 30 byte per second) over RF or direct pin connection ( cable)  

The basic idea is to retransmit every bit PGW64_RETRANSMIT_COUNT times and have a pattern for synchronization between the two (or more) parts of the communication. 

The goal is to be able to transmit maximum PGW64_MAX_BUF_SIZE ( 30 bytes) per second - actually they are transmitted for about 300 millis, but the rest to 1 sec is required to make sure that in the worst case scenario with bad noise the system will be able to resync safely.

Every site is able to receive and transmit, but not in the same time. Transmitting has priority over receiving.

Following is a simple example of sender and receiver code. 

The library also provides function for time based multitasking and timer as it uses the TimerOne and it is not available to the user if the library is in effect.

NOTE: in the real world app every side can be receiver and sender in the same time.


sender :
///////////////////////////////////////////////////////////////////////////////////
// LOG constants
#define LOG64_ENABLED

#include <Log64.h>

  uint64_t LAST_EXECUTE;
  uint32_t TIMEOUT;


  void setup()
  {
    // Serial Log
    LOG64_INIT();

    // PGW64 - TX PIN 11, RX PIN 8 and option POWER PIN 7 to attach the VCC of the RF
    PGW64_INIT(11, 8, 7);

    // Random
    randomSeed(analogRead(0));
    LAST_EXECUTE = 0;
    TIMEOUT = (uint32_t)random(1000, 2000);
  }

  void loop()
  {

    if (PGW64_DO_EXECUTE(PGW64_TIMER_GET(), LAST_EXECUTE, TIMEOUT))
    {
      LAST_EXECUTE = PGW64_TIMER_GET();

      // reset the random timeout between 1 and 2 sec
      TIMEOUT = (uint32_t)random(1000, 2000);

      uint64_t t = PGW64_TIMER_GET();

      LOG64_SET(F("START SENDING... "));
      LOG64_NEW_LINE;

      if (!PGW64_SEND((uint8_t *)"AAAA", strlen("AAAA") + 1))
      //if (!PGW64_SEND((uint8_t *)"123456789A123456789A123456789", strlen("123456789A123456789A123456789") + 1))
      {
        LOG64_SET(F("FAILED TO SEND PACKET."));
        LOG64_NEW_LINE;
      }
      else
      {

        // wait for the packet to be fully sent ( no need to be done in real world application)
        while (PGW64_IS_SENDING())
        {
          delay(1);
        }

        LOG64_SET(F("PACKET FULLY SENT : "));
        LOG64_SET((uint32_t)(PGW64_TIMER_GET() - t));
        LOG64_NEW_LINE;
      }
    }
   
  }

receiver:

///////////////////////////////////////////////////////////////////////////////////
// LOG constants
#define LOG64_ENABLED

#include <Log64.h>

  void setup()
  {
    // Serial Log
    LOG64_INIT();

    // PGW64
    PGW64_INIT(11, 8, 7);
  }

  void loop()
  {

      uint8_t data[PGW64_MAX_BUF_SIZE];
      uint8_t size;

      if (PGW64_RECEIVE(data, size))
      {
        LOG64_SET(F("PACKET RECEIVED ["));
        LOG64_SET((const char *)data);
        LOG64_SET(F("]"));
        LOG64_NEW_LINE;

      } 
  }
