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

#define MAC_DATA_LEN 6

uint8	dev_mac_data[MAC_DATA_LEN]	=	{0x00,0x00,0x00,0x00,0x00,0x00};
static uint8 public_key[] = {
	0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xef,
	0xfe,0xdd,0xcc,0xbb,0xaa,0x99,0x88,0x77,0x66,0x55,0x44,0x33,0x22,0x11
};

void set_addr_from_key(uint8 *dev_mac_data, uint8 *public_key)
{
	//FIXME: order reversed, check!
	dev_mac_data[5] = public_key[0] | 0xc0;
	dev_mac_data[4] = public_key[1];
	dev_mac_data[3] = public_key[2];
	dev_mac_data[2] = public_key[3];
	dev_mac_data[1] = public_key[4];
	dev_mac_data[0] = public_key[5];
};

void set_payload_from_key(uint8_t *payload, uint8_t *public_key) {
	/* copy last 22 bytes */
	osal_memcpy(&payload[7], &public_key[6], 22);
	/* append two bits of public key */
	payload[29] = public_key[0] >> 6;
}

/** Advertisement payload */
static uint8 advertData[] = 
{
	0x1e, /* Length (30) */
	0xff, /* Manufacturer Specific Data (type 0xff) */
	0x4c, 0x00, /* Company ID (Apple) */
	0x12, 0x19, /* Offline Finding type and length */
	0x00, /* State */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, /* First two bits */
	0x00, /* Hint (0x00) */
};

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

void phy_set_channel(uint8 rfChnIdx)
{
    if(g_rfPhyFreqOffSet>=0)
        PHY_REG_WT(0x400300b4, (g_rfPhyFreqOffSet<<16)+(g_rfPhyFreqOffSet<<8)+rfChnIdx);
    else
        PHY_REG_WT(0x400300b4, ((255+g_rfPhyFreqOffSet)<<16)+((255+g_rfPhyFreqOffSet)<<8)+(rfChnIdx-1) );
}

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
    ll_hw_set_trx_settle(   PHYPLUS_HW_BB_DELAY,         // set BB delay
                            PHYPLUS_HW_AFE_DELAY,
                            PHYPLUS_HW_PLL_DELAY);        //RxAFE,PLL
}

void phy_hw_set_stx(void)
{
    ll_hw_set_stx();
    ll_hw_set_trx_settle(   PHYPLUS_HW_BB_DELAY,         // set BB delay
                            PHYPLUS_HW_AFE_DELAY,
                            PHYPLUS_HW_PLL_DELAY);        //RxAFE,PLL
}

void phy_hw_set_trx(uint16 rxTimeOutUs)
{
    ll_hw_set_rx_timeout(rxTimeOutUs);
    ll_hw_set_trx();
    ll_hw_set_trx_settle(   PHYPLUS_HW_BB_DELAY,         // set BB delay
                            PHYPLUS_HW_AFE_DELAY,
                            PHYPLUS_HW_PLL_DELAY);        //RxAFE,PLL
}

void phy_hw_timing_setting(void)
{
    ll_hw_set_tx_rx_release (10,     1);
    ll_hw_set_rx_tx_interval(       60);        //T_IFS=150us for BLE 1M
    ll_hw_set_tx_rx_interval(       66);        //T_IFS=150us for BLE 1M
    ll_hw_set_trx_settle    (57, 8, 52);        //TxBB,RxAFE,PL
}

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

void phy_rf_tx(void)
{
    phy_hw_stop();
    HAL_ENTER_CRITICAL_SECTION();
    phy_hw_pktFmt_Config(s_pktCfg);
    phy_hw_timing_setting();
    phy_set_channel(s_phy.rfChn);

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
    phy_set_channel(s_phy.rfChn);
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
    LOG("HANDLING INTERRUPT\r\n");
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


    set_addr_from_key(dev_mac_data, public_key);
    set_payload_from_key(advertData, public_key);

/// cf. here: https://www.argenox.com/library/bluetooth-low-energy/ble-advertising-primer/
/// First 6 bytes of bluetooth packet are MAC adddress


    s_pubAddr[5] = 0x11;
    s_pubAddr[4] = 0x22;
    s_pubAddr[3] = 0x33;
    s_pubAddr[2] = 0x44;
    s_pubAddr[1] = 0x55;
    s_pubAddr[0] = 0x66;

    //init tx buf
    for(uint8_t i=0; i<255; i++)
        phyBufTx[i]=0;

    uint8_t addressLength = 31;

    uint8_t advHead[2]= {0x00,(addressLength+6)}; 
    //tx buf date update
    phy_tx_buf_updata(dev_mac_data, advHead, advertData, addressLength);
    //phy_tx_buf_updata(s_pubAddr,advHead,adv_buffer,addressLength);
    
    //phy pktfmt config
    s_phy.Status        =   PHYPLUS_RFPHY_IDLE;
    s_phy.txIntv        =   500;//ms
    s_phy.rxIntv        =   200;//ms
    s_phy.rxAckTO       =   500;//us
    s_phy.rxOnlyTO      =   10*1000;//us
    s_phy.rfChn         =   BLE_ADV_CHN37;//26;//
    s_pktCfg.pktFmt     =   PHYPLUS_PKT_FMT_1M;
    s_pktCfg.pduLen     =   31+6;
    s_pktCfg.crcFmt     =   LL_HW_CRC_BLE_FMT;//LL_HW_CRC_BLE_FMT;LL_HW_CRC_NULL
    s_pktCfg.crcSeed    =   DEFAULT_CRC_SEED;
    s_pktCfg.wtSeed     =   WHITEN_SEED_CH37;//DEFAULT_WHITEN_SEED;
    s_pktCfg.syncWord   =   DEFAULT_SYNCWORD;

    VOID osal_start_timerEx(LenzePhy_TaskID, PPP_PERIODIC_TX_EVT, 5000);
}


static void process_rx_done_evt(void)
{
    /**
        37->38->39 scan channel
    */
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
        s_phy.Status = PHYPLUS_RFPHY_IDLE;
    }
}

static void process_tx_done_evt(void)
{

    /**
        37->38->39 adv channel
    */
    if(s_phy.rfChn==BLE_ADV_CHN37)
    {
        LOG("switching channel to 38\r\n");
        s_phy.rfChn = BLE_ADV_CHN38;
        s_pktCfg.wtSeed = WHITEN_SEED_CH38;
        phy_rf_tx();
    }
    else if(s_phy.rfChn==BLE_ADV_CHN38)
    {
       LOG("switching channel to 39\r\n");
        s_phy.rfChn = BLE_ADV_CHN39;
        s_pktCfg.wtSeed = WHITEN_SEED_CH39;
        phy_rf_tx();
    }
    else if(s_phy.rfChn==BLE_ADV_CHN39)
    {
        LOG("switching channel, idle\r\n");
        s_phy.Status = PHYPLUS_RFPHY_IDLE;
    }
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
    VOID task_id;

    if (events & PPP_PERIODIC_TX_EVT)
    {
        if(s_phy.Status==PHYPLUS_RFPHY_IDLE)
        {
            s_phy.Status = PHYPLUS_RFPHY_TX_ONLY;
            s_phy.rfChn = BLE_ADV_CHN37;
            s_pktCfg.wtSeed = WHITEN_SEED_CH37;
            phy_rf_tx();
            osal_start_timerEx(LenzePhy_TaskID,PPP_PERIODIC_TX_EVT,s_phy.txIntv);

        }
        else
        {
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
    /* Initialize the operating system */
    osal_init_system();
    osal_pwrmgr_device(PWRMGR_BATTERY);
    /* Start OSAL */
    osal_start_system(); // No Return from here
    return 0;
}
/*********************************************************************
*********************************************************************/
