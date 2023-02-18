/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/

/**************************************************************************************************
    Filename:       Lenze_phy.c
    Revised:
    Revision:

    Description:    This file contains the phyplus phy sample application


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "rf_phy_driver.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "timer.h"
#include "Lenze_phy.h"
#include "ll.h"
#include "ll_hw_drv.h"
#include "clock.h"
#include "gpio.h"
#include "flash.h"
#include "log.h"
#include "string.h"


/*********************************************************************
    MACROS
*/
#define PHYPLUS_SET_SYNCWORD(x)                 PHY_REG_WT(0x4003004c,(x))
#define PHYPLUS_SET_CRC_SEED(x)                 subWriteReg(0x40030048,23,0,(x))
#define PHYPLUS_SET_WHITEN_SEED(x)              subWriteReg(0x40030048,31,24,(x))

/*********************************************************************
    CONSTANTS
*/
#define PHYPLUS_RFPHY_TX_ONLY                   (0x00)
#define PHYPLUS_RFPHY_RX_ONLY                   (0x01)
#define PHYPLUS_RFPHY_TRX_ONLY                  (0x02)
#define PHYPLUS_RFPHY_RX_TXACK                  (0x03)
#define PHYPLUS_RFPHY_IDLE                      (0xFF)

#define LL_HW_MODE_STX                          (0x00)
#define LL_HW_MODE_SRX                          (0x01)
#define LL_HW_MODE_TRX                          (0x02)


#define PHYPLUS_HW_SCAN_DELAY                   (80)
#define PHYPLUS_HW_BB_DELAY                     (90)
#define PHYPLUS_HW_AFE_DELAY                    ( 8)
#define PHYPLUS_HW_PLL_DELAY                    (60)

#define DEFAULT_CRC_SEED                        (0x555555)

#define DEFAULT_WHITEN_SEED                     (0x37)
#define WHITEN_SEED_CH37                        (0x53)
#define WHITEN_SEED_CH38                        (0x33)
#define WHITEN_SEED_CH39                        (0x73)
#define DEFAULT_WHITEN_SEED                     (0x37)
#define WHITEN_OFF                              (0x00)

#define BLE_ADV_CHN37                           (02)
#define BLE_ADV_CHN38                           (26)
#define BLE_ADV_CHN39                           (80)


#define DEFAULT_SYNCWORD                        (0x8e89bed6)

#define PHYPLUS_PKT_FMT_1M                      (0x01)
#define PHYPLUS_PKT_FMT_2M                      (0x02)
#define PHYPLUS_PKT_FMT_500K                    (0x05)
#define PHYPLUS_PKT_FMT_100K                    (0x06)

#define PHYPLUS_HW_MAX_RX_TO                    (20000)

extern uint8 ll_hw_get_tr_mode(void);
extern volatile uint32 llWaitingIrq;
/*********************************************************************
    TYPE Define
*/
typedef struct pktCfg_s
{
    uint8_t     pktFmt;
    uint8_t     pduLen;
    uint8_t     wtSeed;
    uint8_t     crcFmt;
    uint32_t    crcSeed;
    uint32_t    syncWord;
} pktCfg_t;

typedef struct phyCtx_s
{
    uint8_t     Status;
    uint32_t    txIntv;
    uint32_t    rxIntv;
    uint8_t     rfChn;
    uint16_t    rxOnlyTO;
    uint16_t    rxAckTO;
    uint32_t    rxScanT0;
} phyCtx_t;


/*********************************************************************
    LOCAL VARIABLES
*/
uint8 LenzePhy_TaskID; // Task ID for internal task/event processing
//volatile uint32 phyWaitingIrq = FALSE;
uint32 PHY_ISR_entry_time = 0;

ALIGN4_U8  phyBufRx[256];
ALIGN4_U8  phyBufTx[256];
uint8_t phyBufTxLen;
static uint8_t s_pubAddr[6];
uint8_t adv_buffer[256];


uint16 phyFoff=0;
uint8  phyCarrSens=0;
uint8  phyRssi=0;

static pktCfg_t s_pktCfg;
static phyCtx_t s_phy;
/*********************************************************************
    LOCAL FUNCTIONS
*/
#define MSEC 32 // 32k RTCCounts is about a second (32kHz clock right?)
#define SEC 32768
void debug_blink(uint32_t nblink) {
	gpio_pin_e pin = P3; // LED is on pin 3
	hal_gpioretention_register(pin);
	while(nblink--) {
		hal_gpio_write(pin, 1);
		WaitRTCCount(50*MSEC);
		hal_gpio_write(pin, 0);
		if(nblink) {
			WaitRTCCount(45*MSEC);
		}
	}
}

void switch_mosfet() {
	gpio_pin_e pin = P18; // LED is on pin 3
	static uint8_t state = 1;
	hal_gpioretention_register(pin);
	hal_gpio_write(pin, state);
	state = !state;
}

static uint8_t phy_rx_data_check(void)
{
    if(s_phy.Status==PHYPLUS_RFPHY_RX_TXACK)
    {
        //process data check
        return TRUE;
    }

    osal_set_event(LenzePhy_TaskID,PPP_RX_DATA_PROCESS_EVT);
    return 0;
}

/*
void phy_set_channel(uint8 rfChnIdx)
{
    if(g_rfPhyFreqOffSet>=0)
        PHY_REG_WT(0x400300b4, (g_rfPhyFreqOffSet<<16)+(g_rfPhyFreqOffSet<<8)+rfChnIdx);
    else
        PHY_REG_WT(0x400300b4, ((255+g_rfPhyFreqOffSet)<<16)+((255+g_rfPhyFreqOffSet)<<8)+(rfChnIdx-1) );
}
*/

void phy_hw_go(void)
{

    *(volatile uint32_t*)(LL_HW_BASE+ 0x14) = LL_HW_IRQ_MASK;   //clr  irq status
    *(volatile uint32_t*)(LL_HW_BASE+ 0x0c) = 0x0001;           //mask irq :only use mode done
    *(volatile uint32_t*)(LL_HW_BASE+ 0x00) = 0x0001;           //trig
    uint8_t rfChnIdx = PHY_REG_RD(0x400300b4)&0xff;

    if(rfChnIdx<2)
    {
        rfChnIdx=2;
    }
    else if(rfChnIdx>80)
    {
        rfChnIdx=80;
    }

    if(s_pktCfg.pktFmt==PKT_FMT_BLE2M)
        subWriteReg(0x40030094,7,0,RF_PHY_TPCAL_CALC(g_rfPhyTpCal0_2Mbps,g_rfPhyTpCal1_2Mbps,(rfChnIdx-2)>>1));
    else
        subWriteReg(0x40030094,7,0,RF_PHY_TPCAL_CALC(g_rfPhyTpCal0,g_rfPhyTpCal1,(rfChnIdx-2)>>1));
}


void phy_hw_stop(void)
{
    uint8_t cnt=0;
    ll_hw_set_rx_timeout(33);//will trigger ll_hw_irq=RTO

    while(llWaitingIrq)
    {
        WaitRTCCount(3);
        cnt++;

        if(cnt>10)
        {
            break;
        }
    };
}

void phy_hw_set_srx(uint16 rxTimeOutUs)
{
    ll_hw_set_rx_timeout(rxTimeOutUs);
    ll_hw_set_srx();
    ll_hw_set_trx_settle(LL_HW_BB_DELAY, LL_HW_AFE_DELAY, LL_HW_PLL_DELAY);          //RxAFE,PLL

}

void phy_hw_set_stx(void)
{
    ll_hw_set_stx();
    ll_hw_set_trx_settle(LL_HW_BB_DELAY, LL_HW_AFE_DELAY, LL_HW_PLL_DELAY);          //RxAFE,PLL

}

void phy_hw_set_trx(uint16 rxTimeOutUs)
{
    ll_hw_set_rx_timeout(rxTimeOutUs);
    ll_hw_set_trx();
    ll_hw_set_trx_settle(LL_HW_BB_DELAY, LL_HW_AFE_DELAY, LL_HW_PLL_DELAY);          //RxAFE,PLL

}

static void zb_set_channel (uint8_t chn)
{
    uint32_t rfChnIdx = (chn - 10) * 5;

    if(g_rfPhyFreqOffSet >= 0)
    {
        PHY_REG_WT(0x400300B4, (g_rfPhyFreqOffSet << 16) + (g_rfPhyFreqOffSet << 8) + rfChnIdx);
    }
    else
    {
        PHY_REG_WT(0x400300B4, ((255 + g_rfPhyFreqOffSet) << 16) + ((255 + g_rfPhyFreqOffSet) << 8) + (rfChnIdx - 1) );
    }
    ll_hw_ign_rfifo(LL_HW_IGN_CRC);
}



void phy_hw_timing_setting(void)
{

    ll_hw_set_tx_rx_release     (10, 1);
    ll_hw_set_rx_tx_interval(98);               // T_IFS = 192+2us for ZB 98
    ll_hw_set_tx_rx_interval(108);              // T_IFS = 192-6us for ZB 108
    ll_hw_set_trx_settle(LL_HW_BB_DELAY, LL_HW_AFE_DELAY, LL_HW_PLL_DELAY);    // TxBB, RxAFE, PLL
}

void send_packet()
{
LOG("Sending packet");
    uint8_t m_radio_tx_num = 0;
    phyBufTx[2] = 0x88;
    phyBufTx[3] = m_radio_tx_num++;
    uint8_t pan = 6754;
    #define JOIN_REQUEST 0x01
#define ZIGBEE_BROADCAST_ADDRESS 0xFFFF

uint16_t src = 0x12345987;

phyBufTx[2] = JOIN_REQUEST;
phyBufTx[3] = m_radio_tx_num;
phyBufTx[4] = ((pan >> 0) & (0xFF));
phyBufTx[5] = ((pan >> 8) & (0xFF));
phyBufTx[6] = ((ZIGBEE_BROADCAST_ADDRESS >> 0) & 0xFF);
phyBufTx[7] = ((ZIGBEE_BROADCAST_ADDRESS >> 8) & 0xFF);
phyBufTx[8] = ((src >> 0) & 0xFF);
phyBufTx[9] = ((src >> 8) & 0xFF);
phyBufTx[10] = 0x3F;

uint8_t count = 0; //Payload size
phyBufTx[0] = 11 + count + 2; // hdr, data, crc
    uint16_t total = 1 + 11 + count + 2; // lenb, hdr, data, crc

/*
debug1("Sending join request from %04X", (int)src);
memcpy(&buffer[12], join_payload, join_payload_length);
    // AMID handled below
    //debug1("csnd %04X->%04X[%02X](%d) a:%d", (int)src, (int)dst, (int)amid, (int)count, (int)need_ack);
    memcpy(&phyBufTx[12], comms_get_payload(iface, p_msg, count), count);

    uint32_t evt_time = 0;
    // Pick correct AMID, add timestamp footer when needed
    if (comms_event_time_valid(iface, p_msg))
    {
        phyBufTx[11] = 0x3d; // 3D is used by TinyOS AM for timesync messages
        phyBufTx[12+count] = amid;
        evt_time = comms_get_event_time(iface, p_msg);
        count += 5;
    }
    else
    {
        //debug1("evt time NOT valid");
        phyBufTx[11] = amid;
    }

    phyBufTx[0] = 11 + count + 2; // hdr, data, crc
    uint16_t total = 1 + 11 + count + 2; // lenb, hdr, data, crc

   // tx_timestamps[RADIO_SEND_MSG_PCKT_DONE] = radio_timestamp();
*/
    phyBufTxLen = total;
   rf_tx(total);

}


/*
static uint8_t zbll_hw_read_rfifo_zb (uint8_t* rxPkt, uint16_t* pktLen, uint32_t* pktFoot0, uint32_t* pktFoot1)
{
    int rdPtr, wrPtr, rdDepth, blen, wlen;
    uint32_t* p_rxPkt=(uint32_t*)rxPkt;

    ll_hw_get_rfifo_info(&rdPtr,&wrPtr,&rdDepth);

    if (rdDepth > 0)
    {
        *p_rxPkt++ = *(volatile uint32_t*)(LL_HW_RFIFO);

        blen = rxPkt[0];           //get the byte length for header

        if (blen >= 140) // This is bad, if we don't break out here, the next loop will trash a lot of stuff
        {
            rxPkt[0]  = 0;
            *pktFoot0 = 0;
            *pktFoot1 = 0;
            *pktLen = blen + 1;
            return 0;
        }

        wlen = 1 + ((blen) >> 2);  //blen included the 2byte crc

        while (p_rxPkt < (uint32_t *)rxPkt + wlen)
        {
            *p_rxPkt++ = *(volatile uint32_t*)(LL_HW_RFIFO);
        }

        *pktFoot0 = *(volatile uint32_t*)(LL_HW_RFIFO);
        *pktFoot1 = *(volatile uint32_t*)(LL_HW_RFIFO);

        *pktLen = blen + 1;
        return wlen;
    }
    else
    {
        rxPkt[0]  = 0;
        *pktFoot0 = 0;
        *pktFoot1 = 0;
        *pktLen   = 0;
        return 0;
    }
}
*/


void phy_hw_pktFmt_Config(pktCfg_t cfg)
{
    //baseband cfg
    rf_phy_bb_cfg(cfg.pktFmt);

    //pktfmt
    if(cfg.crcFmt==LL_HW_CRC_NULL)
    {
        //fix length mode ,no hw crc gen/check
        ll_hw_set_pplus_pktfmt(cfg.pduLen);
        ll_hw_ign_rfifo(LL_HW_IGN_NONE);
    }
    else
    {
        //crc
        ll_hw_set_crc_fmt(cfg.crcFmt,cfg.crcFmt);
        PHYPLUS_SET_CRC_SEED(cfg.crcSeed);
        ll_hw_ign_rfifo(LL_HW_IGN_CRC);
    }

    //whiten
    PHYPLUS_SET_WHITEN_SEED(cfg.wtSeed);
    //syncword
    PHYPLUS_SET_SYNCWORD(cfg.syncWord);
}
void rf_tx(uint8_t len)
{
	LOG("HERE");
   ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();
        
    /*
    if (s_phy.Status != PHYPLUS_RFPHY_IDLE)
    {
	    LOG("STOPPING");
        phy_hw_stop();
    }
    */
    LOG("SENDING");
    phy_hw_set_stx();

    uint8_t needAck = 0;
    if (needAck)
    {
    //    radio_tx_wait_ack = true;
        phyBufTx[1] = 0x61;
    }
    else
    {
     //   radio_tx_wait_ack = false;
        phyBufTx[1] = 0x41;
    }

    /*
    if (evt_time != 0)
    {
        uint32_t diff, ti;
        uint16_t count = len - 19;
        ti = radio_timestamp();
        diff = evt_time - ti;

        phyBufTx[13+count] = diff>>24;
        phyBufTx[14+count] = diff>>16;
        phyBufTx[15+count] = diff>>8;
        phyBufTx[16+count] = diff;
    }
    */
    ll_hw_write_tfifo(&phyBufTx[0], len);
    ll_hw_go();

    uint32_t newmode = ll_hw_get_tr_mode();
        //info1("rf : %d md:%d",osKernelGetTickCount(), (int)newmode);
    if (LL_HW_MODE_STX != newmode)
    {
        LOG("mode: %d != %d", (int)newmode, (int)LL_HW_MODE_STX);
    }

}

void phy_rf_tx(void)
{
    phy_hw_stop();
    HAL_ENTER_CRITICAL_SECTION();
    phy_hw_pktFmt_Config(s_pktCfg);
    phy_hw_timing_setting();
    //phy_set_channel(s_phy.rfChn);
    zb_set_channel(11);

    if(s_phy.Status==PHYPLUS_RFPHY_TRX_ONLY)
        phy_hw_set_trx(s_phy.rxAckTO);
    else
        phy_hw_set_stx();

    ll_hw_rst_tfifo();
    ll_hw_rst_rfifo();
    set_max_length(0xff);
    //need updata phyBufTx
    ll_hw_write_tfifo(phyBufTx,phyBufTx[1]+2);
    phy_hw_go();
    llWaitingIrq=TRUE;
    HAL_EXIT_CRITICAL_SECTION();
}

void phy_rf_rx(void)
{
    phy_hw_stop();
    HAL_ENTER_CRITICAL_SECTION();
    phy_hw_pktFmt_Config(s_pktCfg);
    phy_hw_timing_setting();
    //phy_set_channel(s_phy.rfChn);
    zb_set_channel(11);
    phy_hw_set_srx(s_phy.rxOnlyTO);
    ll_hw_rst_tfifo();
    ll_hw_rst_rfifo();
    set_max_length(0xff);
    phy_hw_go();
    llWaitingIrq=TRUE;
    HAL_EXIT_CRITICAL_SECTION();
}

void phy_rx_data_process(void)
{
    uint8_t pduLen=0;

    if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
    {
        pduLen = s_pktCfg.pduLen;
    }
    else
    {
        pduLen = phyBufRx[1];
    }

    {
    uint8_t  len = phyBufRx[0];
    uint8_t noIdea = phyBufRx[1];
    uint8_t* macAddrReverse = &phyBufRx[2];
    uint8_t* packet = macAddrReverse + 6;
    uint8_t* packetLen = packet[0];
    uint8_t* packetType = packet[1];
    uint8_t* packetVal = packet + 2;
		switch_mosfet();
    if(packetType == 0x09 && packetVal[0] == 0x47)
    {
	LOG("TYPE: %02x", packetVal[0]);
	/*
	LOG("BEGIN\n");
        for(uint8_t i=0; i<pduLen; i++)
            LOG("%02x ",packetVal[i]);
	LOG("END");
	LOG("BEGIN3\n");
	for(uint8_t i = 0; i < 6; i++)
	{
		LOG("%02x ", galaxy[i]);
	}
	*/


	    if(!memcmp("Galaxy Tab S4", packetVal, 13))
	    {
		switch_mosfet();
	    }
    }

    /*
        //LOG("[PHY RX] [-%03ddbm %4dKHz %02d CH] ",phyRssi,phyFoff-512,s_phy.rfChn);
	*/
    }

    /*
    uint8_t* macAddr = &phyBufRx[2];
    uint8_t* macAddr + 6;

	*/
}

void phy_tx_buf_updata(uint8_t* adva,uint8_t* txHead,uint8_t* txPayload,uint8_t dlen)
{
    osal_memcpy(&(phyBufTx[0]),&(txHead[0]),2);          //copy tx header
    osal_memcpy(&(phyBufTx[2]),&(adva[0]),6);              //copy AdvA
    osal_memcpy(&(phyBufTx[8]),&(txPayload[0]),dlen);      //copy payload
}


/*******************************************************************************
    @fn          PLUSPHY_IRQHandler

    @brief      Interrupt Request Handler for Link Layer

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      None
*/
void PLUSPHY_IRQHandler(void)
{
LOG("HANDLING INTERRUPT");
    uint8         mode;
    uint32_t      irq_status;
    //uint32_t      T2, delay;
    PHY_ISR_entry_time = read_current_fine_time();
    irq_status = ll_hw_get_irq_status();

    if (!(irq_status & LIRQ_MD))          // only process IRQ of MODE DONE
    {
        ll_hw_clr_irq();                  // clear irq status
        return;
    }
    //debug_blink(1);

    llWaitingIrq = FALSE;
    HAL_ENTER_CRITICAL_SECTION();
    mode = ll_hw_get_tr_mode();

    // ===================   mode TRX process 1
    if (mode == LL_HW_MODE_STX && (s_phy.Status == PHYPLUS_RFPHY_TX_ONLY))
    {
        osal_set_event(LenzePhy_TaskID,PPP_TX_DONE_EVT);
    }
    else if(mode == LL_HW_MODE_SRX && (s_phy.Status == PHYPLUS_RFPHY_RX_ONLY))
    {
        rf_phy_get_pktFoot(&phyRssi,&phyFoff,&phyCarrSens);

        if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
        {
            if(0==(irq_status & LIRQ_RTO))
            {
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                ll_hw_read_rfifo_pplus(phyBufRx, &pktLen,&pktFoot0, &pktFoot1);
                rf_phy_get_pktFoot_fromPkt(pktFoot0,pktFoot1, &phyRssi,&phyFoff,&phyCarrSens);
                phy_rx_data_check();
            }
        }
        else
        {
            if(irq_status & LIRQ_COK)
            {
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                ll_hw_read_rfifo(phyBufRx, &pktLen,&pktFoot0, &pktFoot1);
                rf_phy_get_pktFoot_fromPkt(pktFoot0,pktFoot1, &phyRssi,&phyFoff,&phyCarrSens);
                phy_rx_data_check();
            }
        }

        osal_set_event(LenzePhy_TaskID,PPP_RX_DONE_EVT);
    }
    else if(mode == LL_HW_MODE_TRX  &&
            (s_phy.Status == PHYPLUS_RFPHY_TRX_ONLY)
           )
    {
        rf_phy_get_pktFoot(&phyRssi,&phyFoff,&phyCarrSens);

        if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
        {
            if(0==(irq_status & LIRQ_RTO))
            {
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                ll_hw_read_rfifo_pplus(phyBufRx, &pktLen,&pktFoot0,&pktFoot1);
                rf_phy_get_pktFoot_fromPkt(pktFoot0,pktFoot1,
                                           &phyRssi,&phyFoff,&phyCarrSens);
                phy_rx_data_check();
            }
        }
        else
        {
            if(irq_status & LIRQ_COK)
            {
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                ll_hw_read_rfifo(phyBufRx, &pktLen,&pktFoot0,&pktFoot1);
                rf_phy_get_pktFoot_fromPkt(pktFoot0,pktFoot1,
                                           &phyRssi,&phyFoff,&phyCarrSens);
                phy_rx_data_check();
            }
        }

        osal_set_event(LenzePhy_TaskID,PPP_TRX_DONE_EVT);
    }

    // post ISR process
    ll_hw_clr_irq();
    HAL_EXIT_CRITICAL_SECTION();
}

/*********************************************************************
    @fn      LenzePhy_Init

    @brief   Initialization function for the Simple BLE Peripheral App Task.
            This is called during initialization and should contain
            any application specific initialization (ie. hardware
            initialization/setup, table initialization, power up
            notificaiton ... ).

    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/
void LenzePhy_Init(uint8 task_id)
{
    LenzePhy_TaskID = task_id;
    //set phy irq handeler
    JUMP_FUNCTION(V4_IRQ_HANDLER) = (uint32_t)&PLUSPHY_IRQHandler;

    s_phy.Status        =   PHYPLUS_RFPHY_IDLE;

    s_phy.txIntv        =   500;//ms

    VOID osal_start_timerEx(LenzePhy_TaskID, PPP_PERIODIC_TX_EVT, 1000);
}


static void process_rx_done_evt(void)
{
    /**
        37->38->39 scan channel
    uint32_t t0=read_current_fine_time();

    if(TIME_DELTA(t0, s_phy.rxScanT0)<s_phy.rxOnlyTO)
    {
        phy_rf_rx();
        return;
    }
    else
    {
        //update rx scan time stamp on next chn
        s_phy.rxScanT0 = t0;
    }

    if(s_phy.rfChn==BLE_ADV_CHN37)
    {
        s_phy.rfChn = BLE_ADV_CHN38;
        s_pktCfg.wtSeed = WHITEN_SEED_CH38;
        phy_rf_rx();
    }
    else if(s_phy.rfChn==BLE_ADV_CHN38)
    {
        s_phy.rfChn = BLE_ADV_CHN39;
        s_pktCfg.wtSeed = WHITEN_SEED_CH39;
        phy_rf_rx();
    }
    else if(s_phy.rfChn==BLE_ADV_CHN39)
    {
    }
    */
        s_phy.Status = PHYPLUS_RFPHY_IDLE;
}

static void process_tx_done_evt(void)
{
    /**
        37->38->39 adv channel
    if(s_phy.rfChn==BLE_ADV_CHN37)
    {
        s_phy.rfChn = BLE_ADV_CHN38;
        s_pktCfg.wtSeed = WHITEN_SEED_CH38;
        phy_rf_tx();
    }
    else if(s_phy.rfChn==BLE_ADV_CHN38)
    {
        s_phy.rfChn = BLE_ADV_CHN39;
        s_pktCfg.wtSeed = WHITEN_SEED_CH39;
        phy_rf_tx();
    }
    else if(s_phy.rfChn==BLE_ADV_CHN39)
    {
        s_phy.Status = PHYPLUS_RFPHY_IDLE;
    }
    */
	s_phy.Status = PHYPLUS_RFPHY_IDLE;
}


/*********************************************************************
    @fn      LenzePhy_ProcessEvent

    @brief   Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/
uint16 LenzePhy_ProcessEvent(uint8 task_id, uint16 events)
{
	//LOG("PROCESSEVENT");
    VOID task_id;

    if (events & PPP_PERIODIC_TX_EVT)
    {
        if(s_phy.Status==PHYPLUS_RFPHY_IDLE)
        {
            s_phy.Status = PHYPLUS_RFPHY_TX_ONLY;
            send_packet();
            osal_start_timerEx(LenzePhy_TaskID,PPP_PERIODIC_TX_EVT,s_phy.txIntv);

        }
        else
        {
		LOG("Send again");
	    /// Send the event again.
            osal_start_timerEx(LenzePhy_TaskID,PPP_PERIODIC_TX_EVT,20);
        }

        return(events ^ PPP_PERIODIC_TX_EVT);
    }

    if (events & PPP_PERIODIC_RX_EVT)
    {
        if(s_phy.Status==PHYPLUS_RFPHY_IDLE)
        {
            s_phy.Status = PHYPLUS_RFPHY_RX_ONLY;
            s_phy.rfChn = BLE_ADV_CHN37;
            s_phy.rxScanT0 = read_current_fine_time();
            s_pktCfg.wtSeed = WHITEN_SEED_CH37;
            //rf_rx(phyBufTxLen);
            osal_start_timerEx(LenzePhy_TaskID,PPP_PERIODIC_RX_EVT,s_phy.rxIntv);
        }
        else
        {
            osal_start_timerEx(LenzePhy_TaskID,PPP_PERIODIC_RX_EVT,20);
        }

        return(events ^ PPP_PERIODIC_RX_EVT);
    }

    if(events & PPP_RX_DATA_PROCESS_EVT)
    {
        phy_rx_data_process();
        return(events ^ PPP_RX_DATA_PROCESS_EVT);
    }

    if(events & PPP_TX_DONE_EVT)
    {
        process_tx_done_evt();
        return(events ^ PPP_TX_DONE_EVT);
    }

    if(events & PPP_RX_DONE_EVT)
    {
        process_rx_done_evt();
        return(events ^ PPP_RX_DONE_EVT);
    }

    if(events & PPP_TRX_DONE_EVT)
    {
        //TODO
        return(events ^ PPP_TRX_DONE_EVT);
    }

    return 0;
}

/**************************************************************************************************
    @fn          main

    @brief       Start of application.

    @param       none

    @return      none
 **************************************************************************************************
*/
int app_main(void)
{
    //debug_blink(2);
    /* Initialize the operating system */
    osal_init_system();
    osal_pwrmgr_device(PWRMGR_BATTERY);
    /* Start OSAL */
    osal_start_system(); // No Return from here
    return 0;
}
/*********************************************************************
*********************************************************************/
