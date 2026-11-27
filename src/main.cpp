#include <Arduino.h>
#include <SPI.h>

#include <cc1101.h>
#include <hal/cc1101_hal_arduino.h>
#include <afec_node.h>

#include "afec_platform_arduino.h"
#include "afec_cmd.h"
#include "config.h"

CC1101HalArduino hal(SPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN, 4000000, PIN_GDO0);
CC1101 radio;

afec::Node node;
static CC1101::Config   rcfg;
static afec::NodeConfig ncfg;
static cmd::LineReader  lines;
static uint16_t curBlock = 0;        // from the last `mark` (informational; app_rx carries the payload's block)

void debugPrint(const char *format, ...) {
    char buf[192];
    va_list ap;
    va_start(ap, format);
    int n = vsnprintf(buf, sizeof buf, format, ap);
    va_end(ap);
    Serial.write((const uint8_t*)buf, n < (int)sizeof buf ? n : sizeof buf - 1);
}

// Re-initialise radio and node with the current rcfg/ncfg (emits the library's `begin` event),
// then reset the link to the sender. The mode must match the sender's.
static bool reinit() {
    if (USE_IRQ) radio.disableInterrupt();
    if (!radio.begin(hal, rcfg)) { cmd::ev('E', "radio_err", ",\"err\":\"CC1101 not found\""); return false; }
    if (USE_IRQ && !radio.enableInterrupt()) cmd::ev('W', "irq_fallback");
    node.begin(radio, ncfg, afec::arduinoPlatform());
    node.resetLink(PEER_NODE_ID);
    return true;
}

// One app_rx per delivered (non-duplicate) payload. ok=1: the synthetic payload's CRC32 matches.
static void onData(void*, uint8_t src, const uint8_t* p, uint8_t n, uint8_t fb) {
    uint32_t aseq; uint16_t block, idx;
    const bool ok = cmd::synCheck(p, n, aseq, block, idx);
    cmd::ev('I', "app_rx", ",\"src\":%u,\"len\":%u,\"ok\":%u,\"aseq\":%lu,\"block\":%u,\"idx\":%u,\"fb\":%u",
            src, n, (unsigned)ok, (unsigned long)aseq, block, idx, fb);
}

static void handle(char* line) {
    cmd::Args a;
    if (!a.parse(line)) return;
    const char* c = a.name;

    if (!strcmp(c, "hello")) {
        cmd::ev('I', "hello", ",\"role\":\"receiver\",\"peer\":%u,\"build\":\"%s %s\"", PEER_NODE_ID, __DATE__, __TIME__);
        cmd::printCfg(ncfg, rcfg.txPowerDbm);
        return;
    }
    if (!strcmp(c, "stats")) { cmd::printStats(node.stats()); return; }
    if (!strcmp(c, "mark"))  { cmd::mark(a, curBlock); return; }
    if (!strcmp(c, "cfg")) {
        afec::NodeConfig nc = ncfg; int8_t tx = rcfg.txPowerDbm;
        if (!cmd::parseCfg(a, nc, tx)) return;
        ncfg = nc; rcfg.txPowerDbm = tx;
        if (reinit()) cmd::printCfg(ncfg, rcfg.txPowerDbm);
        return;
    }
    if (!strcmp(c, "reset")) {
        static const char* const keys[] = { "peer", nullptr };
        uint8_t peer = PEER_NODE_ID;
        if (!a.only(keys) || !a.num("peer", 1, 254, peer)) return;
        node.resetLink(peer);
        return;
    }
    cmd::fail(c, "unknown command", "hello|cfg|mark|reset|stats");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    cmd::nodeId() = NODE_ID;

    rcfg.freqMHz    = RF_FREQ_MHZ;
    rcfg.syncWord   = RF_SYNC_WORD;
    rcfg.txPowerDbm = RF_TX_DBM;
    rcfg.whitening  = true;
    rcfg.hwCrc      = false;

    ncfg.id = NODE_ID; ncfg.bitrateBps = 4800;     // everything else: library defaults (C5)

    node.printDebug = debugPrint;

    if (!radio.begin(hal, rcfg)) {
        Serial.println("CC1101 not found - check wiring/power");
        while (true) delay(1000);
    }
    if (USE_IRQ) {
        if (!radio.enableInterrupt()) Serial.println("IRQ setup failed, falling back to polling");
    }
    Serial.printf("CC1101 ready, VERSION=0x%02X, f=%.3f MHz\n", radio.version(), RF_FREQ_MHZ);

    node.begin(radio, ncfg, afec::arduinoPlatform());
    node.onData(onData);
    cmd::ev('I', "hello", ",\"role\":\"receiver\",\"peer\":%u,\"build\":\"%s %s\"", PEER_NODE_ID, __DATE__, __TIME__);
    cmd::printCfg(ncfg, rcfg.txPowerDbm);
}

void loop() {
    node.poll();
    if (char* line = lines.poll()) handle(line);
}
