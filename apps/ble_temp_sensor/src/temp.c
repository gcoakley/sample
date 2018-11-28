#include <nrf_temp.h>

#include "temp.h"
#include "ble_temp_sensor.h"

//#define TEMPTESTDATA
//#define PRINTTEMPREAD
//#define PRINTOTEMP


int16_t TempReadings[TEMP_READINGS_CNT];
int8_t TempReadingsLastIdx;

static struct os_mutex TempMutex;


// Initialize Temperature Readings
void initTempReadings(void)
{
    // Initialize Temperature Readings
    memset(TempReadings, 0x00, sizeof(TempReadings));
    TempReadingsLastIdx = 0;

    // Initialize mutex
    os_mutex_init(&TempMutex);
}

/* Returns the internal temperature of the nRF52 in degC (2 decimal places, scaled) */
int16_t get_temp_measurement(void)
{
#ifndef TEMPTESTDATA
    int16_t temp;

    /* Start the temperature measurement. */
    NRF_TEMP->TASKS_START = 1;
    while(NRF_TEMP->EVENTS_DATARDY != TEMP_INTENSET_DATARDY_Set) {};
    /* Temp reading is in units of 0.25degC, so divide by 4 to get in units of degC
     * (scale by 100 to avoid representing as decimal). */
    temp = (nrf_temp_read() * 100) / 4.0;
#else
    static int16_t temp = 0;

    temp += 1;
#endif

    return temp;
}

// read temperature task
void readTempTask(void *arg)
{
    static os_time_t ms_ostick;

    if((os_time_ms_to_ticks(TEMP_READING_INTERVAL, &ms_ostick)) != 0)
        ms_ostick = 100; //use 100 as fallback if conversion overflows

    while(1)
    {
        os_time_delay(ms_ostick);

        //Lock
        os_mutex_pend(&TempMutex, OS_TIMEOUT_NEVER);

        //Read current temperature & insert into Temperature Array
        TempReadingsLastIdx = ((TempReadingsLastIdx + 1) % TEMP_READINGS_CNT);
        TempReadings[TempReadingsLastIdx] = get_temp_measurement();

        //Release
        os_mutex_release(&TempMutex);

#ifdef PRINTTEMPREAD
        LOG(INFO, "TempRead(%d) = %d\n", TempReadingsLastIdx, TempReadings[TempReadingsLastIdx]);
#endif
    }
}

// Order Temperature Readings (freshest to oldest)
void OrderTempReadingsFIFO(int16_t * TempFO)
{
    int16_t TTempReadings[TEMP_READINGS_CNT];
    int8_t idx, jidx;

    //Lock
    os_mutex_pend(&TempMutex, OS_TIMEOUT_NEVER);

    // make copy of data
    memcpy(TTempReadings, TempReadings, sizeof(TempReadings));
    jidx = TempReadingsLastIdx;

    //Release
    os_mutex_release(&TempMutex);

    for(idx = 0; idx < TEMP_READINGS_CNT; idx++)
    {
        TempFO[idx] = TTempReadings[jidx];
#ifdef PRINTOTEMP
        LOG(INFO, "OTempRead(%d %d) = %d\n", idx, jidx, TempFO[idx]);
#endif
        jidx--;
        if(jidx < 0)
            jidx = TEMP_READINGS_CNT - 1;
    }
}