/*
 * NI USB-6423 plant monitor. Build as C on Windows with
 * NIDAQmx.h on the include path, and link the matching NIDAQmx.lib.
 *
 * Run the program from the directory where you want the CSV files.
 * Ctrl+C finishes the current read block, takes an ending soil snapshot,
 * closes the current CSV, and turns off the outputs.
 *
 * CHANNELS
 *   ai0:4   Differential plant electrodes, with AI 8:12 as the negative leads.
 *   ai16:20 Soil probe voltage outputs, referenced to AI GND.
 *   P0.0    Active-high grow-light switch command.
 *           On from 07:00 to 19:00 local PC time. Use Eastern Time in Atlanta.
 *   P0.1:5  Active-high dry LEDs for plants 1 through 5.
 *   CLI     Press 1 through 5 to log a stimulation for that plant.
 *           Each event is also listed after moisture_end in the same CSV.
 *
 * ACQUISITION ORDER
 *   Initial soil snapshot -> record 2 hours -> boundary soil snapshot -> repeat.
 *   The USB-6423 shares one ADC. Release the plant AI task before starting
 *   soil AI. Reuse the boundary snapshot as the old file's moisture_end and
 *   the new file's moisture_start. There is a short electrode recording gap
 *   during that handoff. Each electrode scan contains only the plant channels.
 *
 * CSV TIME
 *   Read the PC clock once after the warm-up scans for each file's UTC start.
 *   Soil rows use the PC time when the snapshot read completes. Measurement
 *   rows show UTC and filenames show fixed EST. Electrode rows advance from the
 *   file's start using the DAQ's sample rate. time_s keeps the exact scan
 *   spacing, and four rows per second share a UTC stamp at the default rate.
 *   Each plant's analog conversion follows the scan edge at an offset of
 *   its zero-based index / AI convert rate.
 *   CLI events have millisecond timestamps in UTC and fixed EST (UTC-05:00).
 */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <NIDAQmx.h>

/* Edit the settings in this section. */
#define DEVICE "Dev1"  /* Match the device name in NI MAX. */
#define EPHYS_CHANNELS DEVICE "/ai0:4"
#define OUTPUT_LINES DEVICE "/port0/line0:5"

#define NUM_PLANTS 5
/* Plant N uses electrode ai(N-1), soil ai(15+N), and LED P0.N. */
#define NUM_OUTPUT_LINES (1 + NUM_PLANTS)
#define AI_RANGE_V 10.0
#define RECORDING_SECONDS (2 * 60 * 60)  /* Set to 15 for a rollover bench test. (2 * 60 * 60) */
#define SCAN_RATE_HZ 4.0
#define CONVERT_RATE_HZ (NUM_PLANTS * SCAN_RATE_HZ)  /* 20 Hz, 50 ms between plants. */
#define SCANS_PER_READ 10
#define READ_TIMEOUT_S 5.0
#define WRITE_TIMEOUT_S 2.0
#define CSV_PREFIX "plant_"

#define MOISTURE_SCAN_RATE_HZ 4.0
#define MOISTURE_SCANS 4

/* Discard the first full plant scan after switching away from the probes.
 * The two hours of saved data start on the next scan. Set this to 0 to
 * retain the first scan.
 */
#define PLANT_WARMUP_SCANS 1

/* Daily grow-light window in the PC's local time (EST or EDT in Atlanta).
 * On hour is inclusive and off hour is exclusive. Set on earlier than off.
 */
#define LIGHT_ON_HOUR_LOCAL 7
#define LIGHT_OFF_HOUR_LOCAL 19

/* Enter each probe's measured voltage at your desired dry-soil threshold. */
#define DRY_WHEN_HIGH 1  /* Use 0 if this probe type reads lower when dry. */
#define DRY_HYSTERESIS_V 0.05
static const float64 dryCutoffV[NUM_PLANTS] = {2.0, 2.0, 2.0, 2.0, 2.0};

#define UTC_TEXT_SIZE 24
#define UTC_CSV_FORMAT "%Y-%m-%dT%H:%M:%SZ"
#define EST_FILENAME_FORMAT "%Y-%m-%d_%H-%M-%SEST"
#define EST_OFFSET_SECONDS (5 * 60 * 60)

typedef struct {
    time_t utc;
    float64 volts[NUM_PLANTS];
} MoistureSnapshot; // struct to hold the moisture snapshot data, including UTC time and voltage readings for each plant

typedef struct {
    time_t utc;
    unsigned milliseconds;
    unsigned plant;
} Stimulation;

typedef struct {
    Stimulation *events;
    size_t count;
    size_t capacity;
} StimulationLog;

static volatile LONG stopRequested = 0;

/* The console handler runs on a separate Windows thread. Main owns all
 * DAQmx and file operations, including shutdown.
 */
static BOOL WINAPI requestStop(DWORD event)
{
    if (event == CTRL_C_EVENT || event == CTRL_BREAK_EVENT) {
        InterlockedExchange(&stopRequested, 1);
        return TRUE;
    }
    return FALSE;
}

static int shouldStop(void)
{
    return InterlockedCompareExchange(&stopRequested, 0, 0) != 0;
}

static int daqOk(int32 status, const char *operation)
{
    if (DAQmxFailed(status)) {
        char message[2048] = {0};
        DAQmxGetExtendedErrorInfo(message, (uInt32)sizeof message);
        fprintf(stderr, "NI-DAQmx error %ld in %s\n%s\n", (long)status, operation, message);
        return 0;
    }
    return 1;
}

#define DAQ_OK(call) daqOk((call), #call)

static int clearTask(TaskHandle *task)
{
    if (*task != 0) {
        /* ClearTask stops the task and releases its hardware resources. */
        if (!DAQ_OK(DAQmxClearTask(*task)))
            return 0;
        *task = 0;
    }
    return 1;
}


static int readSystemTime(time_t *utc) // passed pointer to store time in
{
    const time_t now = time(NULL); // get current time in seconds since epoch
    if (now == (time_t)-1) {
        fprintf(stderr, "Cannot read the PC clock.\n");
        return 0;
    }
    *utc = now;
    return 1;
}

static int formatUtc(time_t utc, const char *format, char text[UTC_TEXT_SIZE])
{
    struct tm fields;
    if (gmtime_s(&fields, &utc) != 0 || strftime(text, UTC_TEXT_SIZE, format, &fields) == 0) {
        fprintf(stderr, "Cannot format the UTC timestamp.\n");
        return 0;
    }
    return 1;
}

static int writeOutputs(TaskHandle task, uInt8 levels[NUM_OUTPUT_LINES])
{
    /* Each byte is the state of one line. The on-demand task is already started. */
    return DAQ_OK(DAQmxWriteDigitalLines(task, 1, 0, WRITE_TIMEOUT_S,
        DAQmx_Val_GroupByChannel, levels, NULL, NULL));
}


// checks current time to set grow light on/off; checks each plant's soil moisture and sets dry identifier LEDs. 
// States stored in next[], next compared to levels and writeOutputs if difference. Measurements stored in latest->volts[]. 
static int updateOutputs(TaskHandle task, uInt8 levels[NUM_OUTPUT_LINES],
                         const MoistureSnapshot *latest)
{
    time_t now;
    struct tm local;
    uInt8 next[NUM_OUTPUT_LINES];

    if (!readSystemTime(&now))
        return 0;
    if (localtime_s(&local, &now) != 0) {
        fprintf(stderr, "Cannot get the PC's local time for the grow light.\n");
        return 0;
    }

    /* Recheck the local hour at startup and after each read block. The PC's
     * time zone supplies the current daylight saving offset.
     */
    next[0] = (uInt8)(local.tm_hour >= LIGHT_ON_HOUR_LOCAL &&
                      local.tm_hour < LIGHT_OFF_HOUR_LOCAL);

    //checks each plant's soil moisture and turns on the dry LED if the soil is dry, off if wet. Uses hysteresis to avoid flickering.
    for (int plant = 0; plant < NUM_PLANTS; ++plant) {
        /* Positive margin is dry, independent of the probe's polarity.
         * Once lit, the LED clears only after the signal moves back through
         * the hysteresis band. Between snapshots the value stays constant.
         */
        const float64 margin = DRY_WHEN_HIGH
            ? latest->volts[plant] - dryCutoffV[plant]
            : dryCutoffV[plant] - latest->volts[plant];
        const float64 turnOnAt = levels[plant + 1] ? -DRY_HYSTERESIS_V : 0.0;
        next[plant + 1] = (uInt8)(margin >= turnOnAt);
    }

    if (memcmp(levels, next, sizeof next) != 0) {
        if (!writeOutputs(task, next))
            return 0;
        memcpy(levels, next, sizeof next);
    }
    return 1;
}

static int readMoistureSnapshot(MoistureSnapshot *latest)
{
    static const char *const channels[NUM_PLANTS] = {
        DEVICE "/ai16",
        DEVICE "/ai17",
        DEVICE "/ai18",
        DEVICE "/ai19",
        DEVICE "/ai20"
    };
    MoistureSnapshot snapshot = {0};

    /* Finish all four readings on one probe before starting the next. */
    for (int plant = 0; plant < NUM_PLANTS; ++plant) {
        TaskHandle soilTask = 0;
        float64 samples[MOISTURE_SCANS];
        int32 scansRead = 0;
        float64 sum = 0.0;

        const int readOk =
            DAQ_OK(DAQmxCreateTask("soilSnapshot", &soilTask)) &&
            DAQ_OK(DAQmxCreateAIVoltageChan(
                soilTask, channels[plant], "", DAQmx_Val_RSE,
                -AI_RANGE_V, AI_RANGE_V, DAQmx_Val_Volts, NULL)) &&
            DAQ_OK(DAQmxCfgSampClkTiming(
                soilTask, "", MOISTURE_SCAN_RATE_HZ, DAQmx_Val_Rising,
                DAQmx_Val_FiniteSamps, MOISTURE_SCANS)) &&
            DAQ_OK(DAQmxStartTask(soilTask)) &&
            DAQ_OK(DAQmxReadAnalogF64(
                soilTask, MOISTURE_SCANS, READ_TIMEOUT_S,
                DAQmx_Val_GroupByChannel, samples, MOISTURE_SCANS,
                &scansRead, NULL));

        /* Release the ADC before opening the next probe's task. */
        const int clearOk = clearTask(&soilTask);
        if (!readOk || !clearOk)
            return 0;

        if (scansRead != MOISTURE_SCANS) {
            fprintf(stderr,
                    "Incomplete soil reading on %s: expected %d samples, read %ld.\n",
                    channels[plant], MOISTURE_SCANS, (long)scansRead);
            return 0;
        }

        for (int sample = 0; sample < MOISTURE_SCANS; ++sample)
            sum += samples[sample];

        snapshot.volts[plant] = sum / MOISTURE_SCANS;
    }

    /* Timestamp the completed snapshot and publish all five averages together. */
    if (!readSystemTime(&snapshot.utc))
        return 0;

    *latest = snapshot;
    return 1;
}

static int startElectrodeRecording(TaskHandle *aiTask,
                                  uInt64 scansToAcquire, float64 *scanRate,
                                  float64 *convertRate)
{
    /* Finite sampling gives an exact scan count, including any warm-up scans. */
    return DAQ_OK(DAQmxCreateTask("electrodeRecording", aiTask)) &&
        DAQ_OK(DAQmxCreateAIVoltageChan(*aiTask, EPHYS_CHANNELS, "", DAQmx_Val_Diff,
            -AI_RANGE_V, AI_RANGE_V, DAQmx_Val_Volts, NULL)) &&
        DAQ_OK(DAQmxCfgSampClkTiming(*aiTask, "", SCAN_RATE_HZ, DAQmx_Val_Rising,
            DAQmx_Val_FiniteSamps, scansToAcquire)) &&
        DAQ_OK(DAQmxSetAIConvRate(*aiTask, CONVERT_RATE_HZ)) &&
        DAQ_OK(DAQmxStartTask(*aiTask)) &&
        DAQ_OK(DAQmxGetSampClkRate(*aiTask, scanRate)) &&
        DAQ_OK(DAQmxGetAIConvRate(*aiTask, convertRate));
}

static int readElectrodeScans(TaskHandle aiTask, int32 count,
                              float64 plantV[SCANS_PER_READ * NUM_PLANTS])
{
    int32 aiRead = 0;
    /* GroupByScanNumber stores all five electrode voltages together per scan. */
    if (!DAQ_OK(DAQmxReadAnalogF64(aiTask, count, READ_TIMEOUT_S,
            DAQmx_Val_GroupByScanNumber, plantV, SCANS_PER_READ * NUM_PLANTS,
            &aiRead, NULL)))
        return 0;

    if (aiRead != count) {
        fprintf(stderr, "Incomplete electrode scan block: requested %ld, read %ld.\n",
                (long)count, (long)aiRead);
        return 0;
    }
    return 1;
}

/* Include metadata above the rectangular CSV table. With pandas use
 * read_csv(path, comment="#"), then select rows by record_type.
 */
static FILE *openRecording(time_t startUtc, float64 scanRate, float64 convertRate)
{
    char stamp[UTC_TEXT_SIZE];
    char filenameStamp[UTC_TEXT_SIZE];
    char path[128];
    FILE *csv = NULL;
    if (!formatUtc(startUtc, UTC_CSV_FORMAT, stamp) ||
        !formatUtc(startUtc - EST_OFFSET_SECONDS, EST_FILENAME_FORMAT, filenameStamp))
        return NULL;

    for (unsigned suffix = 0; suffix < 1000; ++suffix) {
        const int length = snprintf(path, sizeof path, CSV_PREFIX "%s_%03u.csv",
                                    filenameStamp, suffix);
        if (length < 0 || (size_t)length >= sizeof path) {
            fprintf(stderr, "CSV file name is too long.\n");
            return NULL;
        }

        /* Exclusive creation preserves any older recording with this name. */
        const int error = fopen_s(&csv, path, "wx");
        if (error == 0)
            break;
        if (error != EEXIST) {
            fprintf(stderr, "Cannot create %s (file error %d).\n", path, error);
            return NULL;
        }
    }

    if (csv == NULL) {
        fprintf(stderr, "No available filename for recording starting at %s.\n", stamp);
        return NULL;
    }
    if (fprintf(csv, "# recording_start_utc=%s\n"
                     "# plant_scan_rate_hz=%.9g\n"
                     "# plant_convert_rate_hz=%.9g\n"
                     "record_type,timestamp_utc,time_s,plant1_V,plant2_V,plant3_V,"
                     "plant4_V,plant5_V,soil1_V,soil2_V,soil3_V,soil4_V,soil5_V,"
                     "stim_timestamp_est,stim_plant\n",
                stamp, scanRate, convertRate) < 0) {
        perror("Cannot write CSV header");
        fclose(csv);
        return NULL;
    }
    printf("Recording %s.\n", path);
    return csv;
}

static int writeMoistureRow(FILE *csv, const char *type, const MoistureSnapshot *snapshot)
{
    char stamp[UTC_TEXT_SIZE];
    const float64 *v = snapshot->volts;
    if (!formatUtc(snapshot->utc, UTC_CSV_FORMAT, stamp))
        return 0;

    /* Fill the soil columns and leave the electrode and CLI event fields empty. */
    if (fprintf(csv, "%s,%s,,,,,,,%.9f,%.9f,%.9f,%.9f,%.9f,,\n", type, stamp,
                v[0], v[1], v[2], v[3], v[4]) < 0) {
        perror("Cannot write soil snapshot");
        return 0;
    }
    return 1;
}

static int writeElectrodeRows(FILE *csv, time_t startUtc, float64 scanRate,
                              uInt64 firstIndex, int32 count,
                              const float64 *plantV)
{
    for (int32 scan = 0; scan < count; ++scan) {
        const double timeS = (double)(firstIndex + (uInt64)scan) / scanRate;
        const float64 *v = plantV + (size_t)scan * NUM_PLANTS;
        char stamp[UTC_TEXT_SIZE];
        /* UTC uses whole seconds. time_s retains the subsecond scan spacing. */
        if (!formatUtc(startUtc + (time_t)timeS, UTC_CSV_FORMAT, stamp))
            return 0;

        /* Per-file time starts at 0. */
        if (fprintf(csv, "electrode,%s,%.12g,%.9f,%.9f,%.9f,%.9f,%.9f,,,,,,,\n", stamp,
                    timeS, v[0], v[1], v[2], v[3], v[4]) < 0) {
            perror("Cannot write electrode scan");
            return 0;
        }
    }

    /* Flush each block so a reader can inspect the growing file. */
    if (fflush(csv) != 0) {
        perror("Cannot flush recording");
        return 0;
    }
    return 1;
}

static int writeStimulationRow(FILE *csv, const char *type, const Stimulation *event)
{
    const char *format = "%Y-%m-%dT%H:%M:%S";
    char utc[UTC_TEXT_SIZE];
    char est[UTC_TEXT_SIZE];
    if (!formatUtc(event->utc, format, utc) ||
        !formatUtc(event->utc - EST_OFFSET_SECONDS, format, est))
        return 0;

    /* CLI events have their own rows and leave measurement fields blank. */
    if (fprintf(csv, "%s,%s.%03uZ,,,,,,,,,,,,%s.%03u-05:00,%u\n",
                type, utc, event->milliseconds, est, event->milliseconds, event->plant) < 0) {
        perror("Cannot write stimulation event");
        return 0;
    }
    return 1;
}

static int logStimulation(FILE *csv, StimulationLog *log, unsigned plant)
{
    FILETIME fileTime;
    ULARGE_INTEGER ticks;
    GetSystemTimeAsFileTime(&fileTime);
    ticks.LowPart = fileTime.dwLowDateTime;
    ticks.HighPart = fileTime.dwHighDateTime;
    const Stimulation event = {
        /* FILETIME counts 100 ns ticks from 1601; time_t counts seconds from 1970. */
        (time_t)(ticks.QuadPart / 10000000ULL - 11644473600ULL),
        (unsigned)(ticks.QuadPart / 10000ULL % 1000ULL),
        plant
    };

    if (log->count == log->capacity) {
        if (log->capacity > (size_t)-1 / sizeof *log->events / 2) {
            fprintf(stderr, "Stimulation log is too large.\n");
            return 0;
        }
        const size_t capacity = log->capacity ? log->capacity * 2 : 32;
        Stimulation *events = realloc(log->events, capacity * sizeof *events);
        if (events == NULL) {
            fprintf(stderr, "Cannot allocate stimulation log.\n");
            return 0;
        }
        log->events = events;
        log->capacity = capacity;
    }
    if (!writeStimulationRow(csv, "stimulation", &event) || fflush(csv) != 0) {
        fprintf(stderr, "Cannot save stimulation event.\n");
        return 0;
    }
    log->events[log->count++] = event;
    printf("Logged stimulation for plant %u.\n", plant);
    return 1;
}

static int pollStimulationKeys(HANDLE console, FILE *csv, StimulationLog *log,
                               BOOL held[NUM_PLANTS])
{
    if (console == NULL)
        return 1;

    INPUT_RECORD records[32];
    DWORD pending = 0;
    if (!GetNumberOfConsoleInputEvents(console, &pending)) {
        fprintf(stderr, "Cannot check console input.\n");
        return 0;
    }
    /* Process the events already queued at the block boundary. */
    while (pending != 0) {
        DWORD count = 0;
        const DWORD batchSize = pending < ARRAYSIZE(records) ? pending : ARRAYSIZE(records);
        if (!ReadConsoleInputW(console, records, batchSize, &count)) {
            fprintf(stderr, "Cannot read console input.\n");
            return 0;
        }
        if (count == 0)
            break;
        pending -= count;

        for (DWORD index = 0; index < count; ++index) {
            if (records[index].EventType != KEY_EVENT)
                continue;
            const KEY_EVENT_RECORD *key = &records[index].Event.KeyEvent;
            const WORD code = key->wVirtualKeyCode;
            unsigned plantNumber;
            if (code >= '1' && code < '1' + NUM_PLANTS)
                plantNumber = code - '1' + 1;
            else if (code >= VK_NUMPAD1 && code < VK_NUMPAD1 + NUM_PLANTS)
                plantNumber = code - VK_NUMPAD1 + 1;
            else
                continue;

            /* Windows repeats key-down events while the key stays held. */
            const unsigned plantIndex = plantNumber - 1;
            if (key->bKeyDown && !held[plantIndex] &&
                !logStimulation(csv, log, plantNumber))
                return 0;
            held[plantIndex] = key->bKeyDown;
        }
    }
    return 1;
}

static int waitForElectrodeBlock(TaskHandle aiTask, int32 count)
{
    const ULONGLONG start = GetTickCount64();
    for (;;) {
        uInt32 available = 0;
        if (!DAQ_OK(DAQmxGetReadAvailSampPerChan(aiTask, &available)))
            return 0;
        if (available >= (uInt32)count)
            return 1;
        if (GetTickCount64() - start >= (ULONGLONG)(READ_TIMEOUT_S * 1000.0)) {
            fprintf(stderr, "Timed out waiting for electrode scans.\n");
            return 0;
        }
        Sleep(25);
    }
}

static int closeRecording(FILE **csv)
{
    if (*csv != NULL) {
        const int result = fclose(*csv);
        *csv = NULL;
        if (result != 0) {
            perror("Cannot close recording");
            return 0;
        }
    }
    return 1;
}

int main(void)
{
    TaskHandle aiTask = 0;
    TaskHandle doTask = 0;
    int outputsStarted = 0;
    uInt8 levels[NUM_OUTPUT_LINES] = {0}; //[0] is grow light, [1-5] are dry LEDs for each plant; 0 off, 1 on
    FILE *csv = NULL;
    HANDLE console = GetStdHandle(STD_INPUT_HANDLE);
    DWORD consoleMode;
    BOOL held[NUM_PLANTS] = {0};
    StimulationLog stimLog = {0};
    MoistureSnapshot latestMoisture = {0}; // passed to functions as latest
    
    int result = EXIT_FAILURE;
    const uInt64 scansPerFile = (uInt64)(RECORDING_SECONDS * SCAN_RATE_HZ + 0.5);

    if (scansPerFile == 0) {
        fprintf(stderr, "Set a positive recording duration.\n");
        return EXIT_FAILURE;
    }
    if (!SetConsoleCtrlHandler(requestStop, TRUE)) {
        fprintf(stderr, "Cannot register the Ctrl+C handler.\n");
        return EXIT_FAILURE;
    }
    if (!GetConsoleMode(console, &consoleMode)) {
        console = NULL;
        fprintf(stderr, "No Windows console input. CLI stimulation keys are unavailable.\n");
    }

    /* Keep the digital outputs active when switching between analog input tasks. */
    //lightControl task is both grow light and dry LED. runs in background with analog in and out tasks.
    //Grow light is on from 7am to 7pm local time. Dry LEDs are on when the soil is dry, off when wet.
    if (!DAQ_OK(DAQmxCreateTask("lightControl", &doTask)) || //create lightControl digital output task, puts identifier to doTask
        !DAQ_OK(DAQmxCreateDOChan(doTask, OUTPUT_LINES, "", DAQmx_Val_ChanPerLine)) || //create digital output channel for lightControl task
        !DAQ_OK(DAQmxStartTask(doTask)))
        goto cleanup;
    outputsStarted = 1;
    if (!writeOutputs(doTask, levels) || !readMoistureSnapshot(&latestMoisture))
        goto cleanup;

    while (!shouldStop()) {
        float64 scanRate = 0.0;
        float64 convertRate = 0.0;
        float64 plantV[SCANS_PER_READ * NUM_PLANTS];
        time_t startUtc;
        uInt64 saved = 0;

        if (!updateOutputs(doTask, levels, &latestMoisture) ||
            !startElectrodeRecording(&aiTask, scansPerFile + PLANT_WARMUP_SCANS,
                                     &scanRate, &convertRate))
            goto cleanup;

        /* Read and discard the warm-up scans before the first saved row. */
        for (int32 discarded = 0; discarded < PLANT_WARMUP_SCANS; ++discarded) {
            if (!readElectrodeScans(aiTask, 1, plantV))
                goto cleanup;
        }
        /* Anchor the first retained scan to the current PC time in seconds. */
        if (!readSystemTime(&startUtc))
            goto cleanup;
        csv = openRecording(startUtc, scanRate, convertRate);
        if (csv == NULL || !writeMoistureRow(csv, "moisture_start", &latestMoisture))
            goto cleanup;
        stimLog.count = 0;
        /* Ignore key presses queued while the probes were being sampled. */
        if (console != NULL) {
            if (!FlushConsoleInputBuffer(console)) {
                fprintf(stderr, "Cannot clear console input at recording start.\n");
                goto cleanup;
            }
            memset(held, 0, sizeof held);
            puts("Press 1-5 to log a stimulation. Press Ctrl+C to finish.");
        }

        while (saved < scansPerFile && !shouldStop()) {
            const uInt64 remaining = scansPerFile - saved;
            const int32 count = remaining < SCANS_PER_READ ? (int32)remaining : SCANS_PER_READ;
            if ((console != NULL &&
                 !waitForElectrodeBlock(aiTask, count)) ||
                !readElectrodeScans(aiTask, count, plantV) ||
                !writeElectrodeRows(csv, startUtc, scanRate, saved, count, plantV) ||
                !pollStimulationKeys(console, csv, &stimLog, held))
                goto cleanup;
            saved += (uInt64)count;
            if (!shouldStop() && !updateOutputs(doTask, levels, &latestMoisture))
                goto cleanup;
        }

        /* Release the ADC. At a normal file
         * boundary or Ctrl+C, measure the soil before closing the file.
         * Keep latestMoisture for the next file and for the LEDs. An acquisition
         * error takes the cleanup path and leaves the valid CSV rows intact.
         */
        if (!clearTask(&aiTask) ||
            !readMoistureSnapshot(&latestMoisture) ||
            !writeMoistureRow(csv, "moisture_end", &latestMoisture))
            goto cleanup;
        for (size_t index = 0; index < stimLog.count; ++index) {
            if (!writeStimulationRow(csv, "stimulation_summary", &stimLog.events[index]))
                goto cleanup;
        }
        if (!closeRecording(&csv))
            goto cleanup;

        printf("Closed recording after %llu electrode scans.\n", (unsigned long long)saved);
    }
    result = EXIT_SUCCESS;

cleanup:
    free(stimLog.events);
    if (outputsStarted) {
        uInt8 off[NUM_OUTPUT_LINES] = {0};
        if (!writeOutputs(doTask, off))
            result = EXIT_FAILURE;
    }
    if (!clearTask(&aiTask))
        result = EXIT_FAILURE;
    if (!clearTask(&doTask))
        result = EXIT_FAILURE;
    if (!closeRecording(&csv))
        result = EXIT_FAILURE;
    SetConsoleCtrlHandler(requestStop, FALSE);
    return result;
}
