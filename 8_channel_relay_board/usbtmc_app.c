/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Nathan Conrad
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
 *
 */
#define ACTIVE_LEVEL     0
#define RELAY1_PORT      PORT_PA10
#define RELAY2_PORT      PORT_PA09
#define RELAY3_PORT      PORT_PA11
#define RELAY4_PORT      PORT_PA07
#define RELAY5_PORT      PORT_PA06
#define RELAY6_PORT      PORT_PA17
#define RELAY7_PORT      PORT_PA16
#define RELAY8_PORT      PORT_PA05
#define IDN              "RELAY1:EN 1\nRELAY1:EN?\nhttps://github.com/charkster/relay_usbtmc"
#define IDN_QUERY        "*idn?"
#define RST_CMD          "*rst"
#define RELAY1_EN_CMD    "relay1:en "    // RELAY1:EN ON OFF 1 0
#define RELAY1_EN_QUERY  "relay1:en?"    // RELAY1:EN?
#define RELAY2_EN_CMD    "relay2:en "    // RELAY2:EN ON OFF 1 0
#define RELAY2_EN_QUERY  "relay2:en?"    // RELAY2:EN?
#define RELAY3_EN_CMD    "relay3:en "    // RELAY3:EN ON OFF 1 0
#define RELAY3_EN_QUERY  "relay3:en?"    // RELAY3:EN?
#define RELAY4_EN_CMD    "relay4:en "    // RELAY4:EN ON OFF 1 0
#define RELAY4_EN_QUERY  "relay4:en?"    // RELAY4:EN?
#define RELAY5_EN_CMD    "relay5:en "    // RELAY5:EN ON OFF 1 0
#define RELAY5_EN_QUERY  "relay5:en?"    // RELAY5:EN?
#define RELAY6_EN_CMD    "relay6:en "    // RELAY6:EN ON OFF 1 0
#define RELAY6_EN_QUERY  "relay6:en?"    // RELAY6:EN?
#define RELAY7_EN_CMD    "relay7:en "    // RELAY7:EN ON OFF 1 0
#define RELAY7_EN_QUERY  "relay7:en?"    // RELAY7:EN?
#define RELAY8_EN_CMD    "relay8:en "    // RELAY8:EN ON OFF 1 0
#define RELAY8_EN_QUERY  "relay8:en?"    // RELAY8:EN?

#define END_RESPONSE     "\n"            // USB488

#include <strings.h>
#include <stdlib.h>     /* atoi */
#include <stdio.h>      /* fprintf */
#include "tusb.h"
#include "bsp/board.h"
#include "main.h"
#include "sam.h" /* GPIO */

char * get_value(char *in_string);
void samd21_unique_id( char * id_buff );

#if (CFG_TUD_USBTMC_ENABLE_488)
static usbtmc_response_capabilities_488_t const
#else
static usbtmc_response_capabilities_t const
#endif
tud_usbtmc_app_capabilities  =
{
    .USBTMC_status = USBTMC_STATUS_SUCCESS,
    .bcdUSBTMC = USBTMC_VERSION,
    .bmIntfcCapabilities =
    {
        .listenOnly = 0,
        .talkOnly = 0,
        .supportsIndicatorPulse = 1
    },
    .bmDevCapabilities = {
        .canEndBulkInOnTermChar = 0
    },

#if (CFG_TUD_USBTMC_ENABLE_488)
    .bcdUSB488 = USBTMC_488_VERSION,
    .bmIntfcCapabilities488 =
    {
        .supportsTrigger = 1,
        .supportsREN_GTL_LLO = 0,
        .is488_2 = 1
    },
    .bmDevCapabilities488 =
    {
      .SCPI = 1,
      .SR1 = 0,
      .RL1 = 0,
      .DT1 =0,
    }
#endif
};

#define IEEE4882_STB_QUESTIONABLE (0x08u)
#define IEEE4882_STB_MAV          (0x10u)
#define IEEE4882_STB_SER          (0x20u)
#define IEEE4882_STB_SRQ          (0x40u)

static volatile uint8_t status;

// 0=not query, 1=queried, 2=delay,set(MAV), 3=delay 4=ready?
// (to simulate delay)
static volatile uint16_t queryState = 0;
static volatile uint32_t queryDelayStart;
static volatile uint32_t bulkInStarted;

static volatile bool idnQuery;
static volatile bool rst_cmd;
static volatile bool relay1_en_cmd;
static volatile bool relay1_en_query;
static volatile bool relay2_en_cmd;
static volatile bool relay2_en_query;
static volatile bool relay3_en_cmd;
static volatile bool relay3_en_query;
static volatile bool relay4_en_cmd;
static volatile bool relay4_en_query;
static volatile bool relay5_en_cmd;
static volatile bool relay5_en_query;
static volatile bool relay6_en_cmd;
static volatile bool relay6_en_query;
static volatile bool relay7_en_cmd;
static volatile bool relay7_en_query;
static volatile bool relay8_en_cmd;
static volatile bool relay8_en_query;

static uint32_t resp_delay = 125u; // Adjustable delay, to allow for better testing
static size_t   buffer_len;
static size_t   buffer_tx_ix;      // for transmitting using multiple transfers
static uint8_t  buffer[225];       // A few packets long should be enough.

char relay1_en_str[2];
char relay2_en_str[2];
char relay3_en_str[2];
char relay4_en_str[2];
char relay5_en_str[2];
char relay6_en_str[2];
char relay7_en_str[2];
char relay8_en_str[2];


static usbtmc_msg_dev_dep_msg_in_header_t rspMsg = {
    .bmTransferAttributes =
    {
      .EOM = 1,
      .UsingTermChar = 0
    }
};

void tud_usbtmc_open_cb(uint8_t interface_id)
{
  (void)interface_id;
  tud_usbtmc_start_bus_read();
}

#if (CFG_TUD_USBTMC_ENABLE_488)
usbtmc_response_capabilities_488_t const *
#else
usbtmc_response_capabilities_t const *
#endif
tud_usbtmc_get_capabilities_cb()
{
  return &tud_usbtmc_app_capabilities;
}


bool tud_usbtmc_msg_trigger_cb(usbtmc_msg_generic_t* msg) {
  (void)msg;
  // Let trigger set the SRQ
  status |= IEEE4882_STB_SRQ;
  return true;
}

bool tud_usbtmc_msgBulkOut_start_cb(usbtmc_msg_request_dev_dep_out const * msgHeader)
{
  (void)msgHeader;
  buffer_len = 0;
  if(msgHeader->TransferSize > sizeof(buffer))
  {

    return false;
  }
  return true;
}

bool tud_usbtmc_msg_data_cb(void *data, size_t len, bool transfer_complete)
{
  // If transfer isn't finished, we just ignore it (for now)

  if(len + buffer_len < sizeof(buffer))
  {
    memcpy(&(buffer[buffer_len]), data, len);
    buffer_len += len;
  }
  else
  {
    return false; // buffer overflow!
  }

  queryState      = transfer_complete;
  idnQuery        = false;
  rst_cmd         = false;
  relay1_en_cmd   = false;
  relay1_en_query = false;
  relay2_en_cmd   = false;
  relay2_en_query = false;
  relay3_en_cmd   = false;
  relay3_en_query = false;
  relay4_en_cmd   = false;
  relay4_en_query = false;
  relay5_en_cmd   = false;
  relay5_en_query = false;
  relay6_en_cmd   = false;
  relay6_en_query = false;
  relay7_en_cmd   = false;
  relay7_en_query = false;
  relay8_en_cmd   = false;
  relay8_en_query = false;

  if(transfer_complete && (len >=4) && !strncasecmp(IDN_QUERY,data,5))
  {
    idnQuery = true;
  }
  else if (transfer_complete && (len >=4) && !strncasecmp(RST_CMD,data,4))
  {
    rst_cmd = true;
    DAC->DATA.reg = 0x0000;                // clear DAC value
    PORT->Group[0].DIRSET.reg = RELAY1_PORT |  RELAY2_PORT | RELAY3_PORT | RELAY4_PORT | RELAY5_PORT | RELAY6_PORT | RELAY7_PORT | RELAY8_PORT; // as output
    if ( ACTIVE_LEVEL == 1)
    {
      PORT->Group[0].OUTCLR.reg = RELAY1_PORT |  RELAY2_PORT | RELAY3_PORT | RELAY4_PORT | RELAY5_PORT | RELAY6_PORT | RELAY7_PORT | RELAY8_PORT; // drive low values
    }
    else
    {
      PORT->Group[0].OUTSET.reg = RELAY1_PORT |  RELAY2_PORT | RELAY3_PORT | RELAY4_PORT | RELAY5_PORT | RELAY6_PORT | RELAY7_PORT | RELAY8_PORT; // drive high values
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY1_EN_CMD,data,10))
  {
    relay1_en_cmd = true;
    char *ptr_value = get_value(data);
    int relay1_en = atoi(ptr_value);
    if ((relay1_en == 1 && ACTIVE_LEVEL == 1) || (relay1_en == 0 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTSET.reg = RELAY1_PORT; // drive high value
    }
    else if ((relay1_en == 0 && ACTIVE_LEVEL == 1) || ( relay1_en == 1 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTCLR.reg = RELAY1_PORT; // drive low value
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY1_EN_QUERY,data,10))
  {
    relay1_en_query = true;
    if ((PORT->Group[0].DIR.reg & RELAY1_PORT) && ((((PORT->Group[0].OUT.reg & RELAY1_PORT) && ACTIVE_LEVEL == 1)) || ((!(PORT->Group[0].OUT.reg & RELAY1_PORT) && ACTIVE_LEVEL == 0))))
    {
      strcpy(relay1_en_str,"1");
    }
    else
    {
      strcpy(relay1_en_str,"0");
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY2_EN_CMD,data,10))
  {
    relay2_en_cmd = true;
    char *ptr_value = get_value(data);
    int relay2_en = atoi(ptr_value);
    if ((relay2_en == 1 && ACTIVE_LEVEL == 1) || (relay2_en == 0 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTSET.reg =  RELAY2_PORT; // drive high value
    }
    else if ((relay2_en == 0 && ACTIVE_LEVEL == 1) || ( relay2_en == 1 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTCLR.reg =  RELAY2_PORT; // drive low value
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY2_EN_QUERY,data,10))
  {
    relay2_en_query = true;
    if ((PORT->Group[0].DIR.reg & RELAY2_PORT) && ((((PORT->Group[0].OUT.reg & RELAY2_PORT) && ACTIVE_LEVEL == 1)) || ((!(PORT->Group[0].OUT.reg & RELAY2_PORT) && ACTIVE_LEVEL == 0))))
    {
      strcpy(relay2_en_str,"1");
    }
    else
    {
      strcpy(relay2_en_str,"0");
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY3_EN_CMD,data,10))
  {
    relay3_en_cmd = true;
    char *ptr_value = get_value(data);
    int relay3_en = atoi(ptr_value);
    if ((relay3_en == 1 && ACTIVE_LEVEL == 1) || (relay3_en == 0 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTSET.reg = RELAY3_PORT; // drive high value
    }
    else if ((relay3_en == 0 && ACTIVE_LEVEL == 1) || ( relay3_en == 1 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTCLR.reg = RELAY3_PORT; // drive low value
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY3_EN_QUERY,data,10))
  {
    relay3_en_query = true;
    if ((PORT->Group[0].DIR.reg & RELAY3_PORT) && ((((PORT->Group[0].OUT.reg & RELAY3_PORT) && ACTIVE_LEVEL == 1)) || ((!(PORT->Group[0].OUT.reg & RELAY3_PORT) && ACTIVE_LEVEL == 0))))
    {
      strcpy(relay3_en_str,"1");
    }
    else
    {
      strcpy(relay3_en_str,"0");
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY4_EN_CMD,data,10))
  {
    relay4_en_cmd = true;
    char *ptr_value = get_value(data);
    int relay4_en = atoi(ptr_value);
    if ((relay4_en == 1 && ACTIVE_LEVEL == 1) || (relay4_en == 0 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTSET.reg =  RELAY4_PORT; // drive high value
    }
    else if ((relay4_en == 0 && ACTIVE_LEVEL == 1) || ( relay4_en == 1 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTCLR.reg =  RELAY4_PORT; // drive low value
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY4_EN_QUERY,data,10))
  {
    relay4_en_query = true;
    if ((PORT->Group[0].DIR.reg & RELAY4_PORT) && ((((PORT->Group[0].OUT.reg & RELAY4_PORT) && ACTIVE_LEVEL == 1)) || ((!(PORT->Group[0].OUT.reg & RELAY4_PORT) && ACTIVE_LEVEL == 0))))
    {
      strcpy(relay4_en_str,"1");
    }
    else
    {
      strcpy(relay4_en_str,"0");
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY5_EN_CMD,data,10))
  {
    relay5_en_cmd = true;
    char *ptr_value = get_value(data);
    int relay5_en = atoi(ptr_value);
    if ((relay5_en == 1 && ACTIVE_LEVEL == 1) || (relay5_en == 0 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTSET.reg = RELAY5_PORT; // drive high value
    }
    else if ((relay5_en == 0 && ACTIVE_LEVEL == 1) || ( relay5_en == 1 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTCLR.reg = RELAY5_PORT; // drive low value
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY5_EN_QUERY,data,10))
  {
    relay5_en_query = true;
    if ((PORT->Group[0].DIR.reg & RELAY5_PORT) && ((((PORT->Group[0].OUT.reg & RELAY5_PORT) && ACTIVE_LEVEL == 1)) || ((!(PORT->Group[0].OUT.reg & RELAY5_PORT) && ACTIVE_LEVEL == 0))))
    {
      strcpy(relay5_en_str,"1");
    }
    else
    {
      strcpy(relay5_en_str,"0");
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY6_EN_CMD,data,10))
  {
    relay6_en_cmd = true;
    char *ptr_value = get_value(data);
    int relay6_en = atoi(ptr_value);
    if ((relay6_en == 1 && ACTIVE_LEVEL == 1) || (relay6_en == 0 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTSET.reg =  RELAY6_PORT; // drive high value
    }
    else if ((relay6_en == 0 && ACTIVE_LEVEL == 1) || ( relay6_en == 1 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTCLR.reg =  RELAY6_PORT; // drive low value
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY6_EN_QUERY,data,10))
  {
    relay6_en_query = true;
    if ((PORT->Group[0].DIR.reg & RELAY6_PORT) && ((((PORT->Group[0].OUT.reg & RELAY6_PORT) && ACTIVE_LEVEL == 1)) || ((!(PORT->Group[0].OUT.reg & RELAY6_PORT) && ACTIVE_LEVEL == 0))))
    {
      strcpy(relay6_en_str,"1");
    }
    else
    {
      strcpy(relay6_en_str,"0");
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY7_EN_CMD,data,10))
  {
    relay7_en_cmd = true;
    char *ptr_value = get_value(data);
    int relay7_en = atoi(ptr_value);
    if ((relay7_en == 1 && ACTIVE_LEVEL == 1) || (relay7_en == 0 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTSET.reg = RELAY7_PORT; // drive high value
    }
    else if ((relay7_en == 0 && ACTIVE_LEVEL == 1) || ( relay7_en == 1 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTCLR.reg = RELAY7_PORT; // drive low value
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY7_EN_QUERY,data,10))
  {
    relay7_en_query = true;
    if ((PORT->Group[0].DIR.reg & RELAY7_PORT) && ((((PORT->Group[0].OUT.reg & RELAY7_PORT) && ACTIVE_LEVEL == 1)) || ((!(PORT->Group[0].OUT.reg & RELAY7_PORT) && ACTIVE_LEVEL == 0))))
    {
      strcpy(relay7_en_str,"1");
    }
    else
    {
      strcpy(relay7_en_str,"0");
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY8_EN_CMD,data,10))
  {
    relay8_en_cmd = true;
    char *ptr_value = get_value(data);
    int relay8_en = atoi(ptr_value);
    if ((relay8_en == 1 && ACTIVE_LEVEL == 1) || ( relay8_en == 0 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTSET.reg =  RELAY8_PORT; // drive high value
    }
    else if ((relay8_en == 0 && ACTIVE_LEVEL == 1) || ( relay8_en == 1 && ACTIVE_LEVEL == 0))
    {
      PORT->Group[0].OUTCLR.reg =  RELAY8_PORT; // drive low value
    }
  }
  else if (transfer_complete && (len >= 12) && !strncasecmp(RELAY8_EN_QUERY,data,10))
  {
    relay8_en_query = true;
    if ((PORT->Group[0].DIR.reg & RELAY8_PORT) && ((((PORT->Group[0].OUT.reg & RELAY8_PORT) && ACTIVE_LEVEL == 1)) || ((!(PORT->Group[0].OUT.reg & RELAY8_PORT) && ACTIVE_LEVEL == 0))))
    {
      strcpy(relay8_en_str,"1");
    }
    else
    {
      strcpy(relay8_en_str,"0");
    }
  }

  if(transfer_complete && !strncasecmp("delay ",data,5))
  {
    queryState = 0;
    int d = atoi((char*)data + 5);
    if(d > 10000)
      d = 10000;
    if(d<0)
      d=0;
    resp_delay = (uint32_t)d;
  }
  tud_usbtmc_start_bus_read();
  return true;
}

bool tud_usbtmc_msgBulkIn_complete_cb()
{
  if((buffer_tx_ix == buffer_len) || idnQuery) // done
  {
    status &= (uint8_t)~(IEEE4882_STB_MAV); // clear MAV
    queryState = 0;
    bulkInStarted = 0;
    buffer_tx_ix = 0;
  }
  tud_usbtmc_start_bus_read();

  return true;
}

static unsigned int msgReqLen;

bool tud_usbtmc_msgBulkIn_request_cb(usbtmc_msg_request_dev_dep_in const * request)
{
  rspMsg.header.MsgID = request->header.MsgID,
  rspMsg.header.bTag = request->header.bTag,
  rspMsg.header.bTagInverse = request->header.bTagInverse;
  msgReqLen = request->TransferSize;

#ifdef xDEBUG
  uart_tx_str_sync("MSG_IN_DATA: Requested!\r\n");
#endif
  if(queryState == 0 || (buffer_tx_ix == 0))
  {
    TU_ASSERT(bulkInStarted == 0);
    bulkInStarted = 1;

    // > If a USBTMC interface receives a Bulk-IN request prior to receiving a USBTMC command message
    //   that expects a response, the device must NAK the request (*not stall*)
  }
  else
  {
    size_t txlen = tu_min32(buffer_len-buffer_tx_ix,msgReqLen);
    tud_usbtmc_transmit_dev_msg_data(&buffer[buffer_tx_ix], txlen,
        (buffer_tx_ix+txlen) == buffer_len, false);
    buffer_tx_ix += txlen;
  }
  // Always return true indicating not to stall the EP.
  return true;
}

void usbtmc_app_task_iter(void) {
  switch(queryState) {
  case 0:
    break;
  case 1:
    queryDelayStart = board_millis();
    queryState = 2;
    break;
  case 2:
    if( (board_millis() - queryDelayStart) > resp_delay) {
      queryDelayStart = board_millis();
      queryState=3;
      status |= 0x10u; // MAV
      status |= 0x40u; // SRQ
    }
    break;
  case 3:
    if( (board_millis() - queryDelayStart) > resp_delay) {
      queryState = 4;
    }
    break;
  case 4: // time to transmit;
    if(bulkInStarted && (buffer_tx_ix == 0)) {
      if(idnQuery)
      {
//        char unique_id[34] = "";
//        char idn_str[52] = IDN;
//        samd21_unique_id(unique_id);
//        strcat(idn_str,unique_id);
//        tud_usbtmc_transmit_dev_msg_data(idn_str, tu_min32(sizeof(idn_str)-1,msgReqLen),true,false);
        tud_usbtmc_transmit_dev_msg_data(IDN, tu_min32(sizeof(IDN)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (relay1_en_query)
      {
        tud_usbtmc_transmit_dev_msg_data(relay1_en_str, tu_min32(sizeof(relay1_en_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (relay2_en_query)
      {
        tud_usbtmc_transmit_dev_msg_data(relay2_en_str, tu_min32(sizeof(relay2_en_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (relay3_en_query)
      {
        tud_usbtmc_transmit_dev_msg_data(relay3_en_str, tu_min32(sizeof(relay3_en_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (relay4_en_query)
      {
        tud_usbtmc_transmit_dev_msg_data(relay4_en_str, tu_min32(sizeof(relay4_en_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }

      else if (relay5_en_query)
      {
        tud_usbtmc_transmit_dev_msg_data(relay5_en_str, tu_min32(sizeof(relay5_en_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (relay6_en_query)
      {
        tud_usbtmc_transmit_dev_msg_data(relay6_en_str, tu_min32(sizeof(relay6_en_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (relay7_en_query)
      {
        tud_usbtmc_transmit_dev_msg_data(relay7_en_str, tu_min32(sizeof(relay7_en_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (relay8_en_query)
      {
        tud_usbtmc_transmit_dev_msg_data(relay8_en_str, tu_min32(sizeof(relay8_en_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }

      else if (rst_cmd || relay1_en_cmd || relay2_en_cmd || relay3_en_cmd || relay4_en_cmd || relay5_en_cmd || relay6_en_cmd || relay7_en_cmd || relay8_en_cmd)
      { 
        tud_usbtmc_transmit_dev_msg_data(END_RESPONSE, tu_min32(sizeof(END_RESPONSE)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else
      {
        buffer_tx_ix = tu_min32(buffer_len,msgReqLen);
        tud_usbtmc_transmit_dev_msg_data(buffer, buffer_tx_ix, buffer_tx_ix == buffer_len, false);
      }

      // MAV is cleared in the transfer complete callback.
    }
    break;
  default:
    TU_ASSERT(false,);
    return;
  }
}

bool tud_usbtmc_initiate_clear_cb(uint8_t *tmcResult)
{
  *tmcResult = USBTMC_STATUS_SUCCESS;
  queryState = 0;
  bulkInStarted = false;
  status = 0;
  return true;
}

bool tud_usbtmc_check_clear_cb(usbtmc_get_clear_status_rsp_t *rsp)
{
  queryState = 0;
  bulkInStarted = false;
  status = 0;
  buffer_tx_ix = 0u;
  buffer_len = 0u;
  rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
  rsp->bmClear.BulkInFifoBytes = 0u;
  return true;
}
bool tud_usbtmc_initiate_abort_bulk_in_cb(uint8_t *tmcResult)
{
  bulkInStarted = 0;
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}
bool tud_usbtmc_check_abort_bulk_in_cb(usbtmc_check_abort_bulk_rsp_t *rsp)
{
  (void)rsp;
  tud_usbtmc_start_bus_read();
  return true;
}

bool tud_usbtmc_initiate_abort_bulk_out_cb(uint8_t *tmcResult)
{
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;

}
bool tud_usbtmc_check_abort_bulk_out_cb(usbtmc_check_abort_bulk_rsp_t *rsp)
{
  (void)rsp;
  tud_usbtmc_start_bus_read();
  return true;
}

void tud_usbtmc_bulkIn_clearFeature_cb(void)
{
}
void tud_usbtmc_bulkOut_clearFeature_cb(void)
{
  tud_usbtmc_start_bus_read();
}

// Return status byte, but put the transfer result status code in the rspResult argument.
uint8_t tud_usbtmc_get_stb_cb(uint8_t *tmcResult)
{
  uint8_t old_status = status;
  status = (uint8_t)(status & ~(IEEE4882_STB_SRQ)); // clear SRQ

  *tmcResult = USBTMC_STATUS_SUCCESS;
  // Increment status so that we see different results on each read...

  return old_status;
}

bool tud_usbtmc_indicator_pulse_cb(tusb_control_request_t const * msg, uint8_t *tmcResult)
{
  (void)msg;
  led_indicator_pulse();
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}

//---------------------------- New Code ----------------------------//

void gpio_setup(void) {
  PORT->Group[0].DIRSET.reg = RELAY1_PORT | RELAY2_PORT | RELAY3_PORT | RELAY4_PORT | RELAY5_PORT | RELAY6_PORT | RELAY7_PORT | RELAY8_PORT; // as output
  if (ACTIVE_LEVEL == 1) 
  {
    PORT->Group[0].OUTCLR.reg = RELAY1_PORT | RELAY2_PORT | RELAY3_PORT | RELAY4_PORT | RELAY5_PORT | RELAY6_PORT | RELAY7_PORT | RELAY8_PORT; // initialized low
  }
  else
  {
    PORT->Group[0].OUTSET.reg = RELAY1_PORT | RELAY2_PORT | RELAY3_PORT | RELAY4_PORT | RELAY5_PORT | RELAY6_PORT | RELAY7_PORT | RELAY8_PORT; // initialized high
  }
}

char * get_value(char *in_string) {
  char *ptr = strrchr(in_string,' ') + 1;
  return ptr;
}

char * get_command(char *in_string, char *ptr_value) {
  uint32_t command_len = ptr_value - in_string - 1;
  char *command = (char *) malloc(command_len +1);
  memcpy(command, in_string, command_len);
  command[command_len] = '\0';
  return command;
}

void samd21_unique_id( char * id_buff )
{   
    volatile uint32_t val0, val1, val2, val3;
    volatile uint32_t *val0_ptr = (volatile uint32_t *)0x0080A00C;
    volatile uint32_t *val1_ptr = (volatile uint32_t *)0x0080A040;
    volatile uint32_t *val2_ptr = (volatile uint32_t *)0x0080A044;
    volatile uint32_t *val3_ptr = (volatile uint32_t *)0x0080A048;
    val0 = *val0_ptr;
    val1 = *val1_ptr;
    val2 = *val2_ptr;
    val3 = *val3_ptr;
    static char format[] = "0x%08x%08x%08x%08x";
    sprintf(id_buff, format,val0, val1, val2, val3);
}
