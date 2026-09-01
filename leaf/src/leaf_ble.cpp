/*
 * Leaf BLE layer — see leaf_ble.h for the design and the single-radio handoff rule.
 */
#ifdef LEAF_BLE

#include "leaf_ble.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_mac.h"

static BLEServer*         s_server  = nullptr;
static BLECharacteristic* s_status  = nullptr;
static BLEAdvertising*    s_adv     = nullptr;
static bool               s_active  = false;
static volatile bool      s_conn    = false;
// Set for the duration of leaf_ble_end(). deinit() drops any live connection, which fires
// onDisconnect from the NimBLE host task while the server/advertising objects are being
// freed underneath it — re-arming advertising there is a use-after-free. Found on the
// bench 2026-08-17: writing "wifi" over GATT tears the stack down with the client still
// connected and paniced with "CORRUPT HEAP: Bad head ... multi_heap_free". The unattended
// path (window expires, nobody connected) never hit it, which is why it took a real GATT
// client to surface.
static volatile bool      s_stopping = false;
// The live connection's handle. NOT zero: the first NimBLE connection on this build comes
// up as handle 1, so the obvious s_server->disconnect(0) fails with rc=7 (BLE_HS_ENOTCONN)
// and silently leaves the link up — which is how the first teardown fix still paniced.
// 0xFFFF = no connection.
static volatile uint16_t  s_conn_handle = 0xFFFF;
static volatile LeafBleCmd s_cmd    = LEAF_BLE_CMD_NONE;
static char               s_name[24] = {0};
static String             s_payload  = "{}";

// --- callbacks -------------------------------------------------------------------------
// Keep these SHORT. They run on the NimBLE host task; anything that blocks here stalls the
// stack, and re-entering the app's own radio/camera work from inside one is the same class
// of bug as re-entering reticulum.loop() from a packet callback (leaf_radio_poll_commands).
class LeafBleServerCallbacks : public BLEServerCallbacks {
    // NimBLE hands us the connection descriptor; that is the only place the real handle is
    // available, and leaf_ble_end() needs it to terminate the link before deinit.
    void onConnect(BLEServer* srv, ble_gap_conn_desc* desc) override {
        s_conn        = true;
        s_conn_handle = desc ? desc->conn_handle : 0xFFFF;
        Serial.printf("[ble] phone connected (handle=%u)\n", (unsigned)s_conn_handle);
    }
    void onConnect(BLEServer* srv) override {
        s_conn = true;   // handle-less overload; the one above carries the handle
    }
    void onDisconnect(BLEServer* srv, ble_gap_conn_desc* desc) override {
        s_conn_handle = 0xFFFF;
        onDisconnect(srv);
    }
    void onDisconnect(BLEServer* srv) override {
        s_conn_handle = 0xFFFF;
        s_conn = false;
        if (s_stopping) {          // teardown in progress: touch NOTHING, it is being freed
            Serial.println("[ble] disconnected during shutdown");
            return;
        }
        Serial.println("[ble] phone disconnected -> re-advertising");
        // NimBLE does not auto-restart advertising after a disconnect; without this the
        // node goes invisible after the first walk-up of the wake.
        if (s_adv) s_adv->start();
    }
};

class LeafBleStatusCallbacks : public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic* chr) override {
        // Value is already staged by leaf_ble_set_status(); nothing to recompute. Logged
        // because "did the phone actually READ, or only connect" is the exact question
        // the fit test has to answer.
        Serial.printf("[ble] read: %u bytes\n", (unsigned)chr->getLength());
    }
    void onWrite(BLECharacteristic* chr) override {
        String v = chr->getValue();
        v.trim();
        LeafBleCmd c = LEAF_BLE_CMD_UNKNOWN;
        if      (v.equalsIgnoreCase("wifi"))  c = LEAF_BLE_CMD_WIFI;
        else if (v.equalsIgnoreCase("sleep")) c = LEAF_BLE_CMD_SLEEP;
        s_cmd = c;
        Serial.printf("[ble] write: \"%s\" -> cmd=%d\n", v.c_str(), (int)c);
        // Restore the status payload so the next read doesn't hand back the command echo.
        chr->setValue(s_payload);
    }
};

static LeafBleServerCallbacks s_srv_cb;
static LeafBleStatusCallbacks s_chr_cb;

// --- API -------------------------------------------------------------------------------
bool leaf_ble_begin(const char* name_suffix) {
    if (s_active) return true;

    if (name_suffix && *name_suffix) {
        snprintf(s_name, sizeof(s_name), "MeshCam-%s", name_suffix);
    } else {
        uint8_t mac[6] = {0};
        esp_read_mac(mac, ESP_MAC_BT);
        snprintf(s_name, sizeof(s_name), "MeshCam-%02X%02X%02X", mac[3], mac[4], mac[5]);
    }

    if (!BLEDevice::init(s_name)) {
        Serial.println("[ble] BLEDevice::init FAILED");
        s_name[0] = 0;
        return false;
    }
    // 247 is the largest the NimBLE host will negotiate here; the status JSON fits in the
    // 23-byte default, but a full walk-up exchange (the WiFi credential handoff)
    // will not, and the MTU is negotiated at connect time — too late to raise later.
    BLEDevice::setMTU(247);

    s_server = BLEDevice::createServer();
    s_server->setCallbacks(&s_srv_cb);

    BLEService* svc = s_server->createService(LEAF_BLE_SERVICE_UUID);
    s_status = svc->createCharacteristic(
        LEAF_BLE_STATUS_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY);
    s_status->setCallbacks(&s_chr_cb);
    s_status->setValue(s_payload);
    svc->start();

    // Advertise the service UUID so the app can filter its scan; the name goes in the scan
    // response because a 128-bit UUID (18 B) plus flags (3 B) leaves only 10 B in the 31 B
    // advertising PDU — not enough for "MeshCam-XXXXXX" (16 B with its header).
    s_adv = BLEDevice::getAdvertising();
    s_adv->addServiceUUID(LEAF_BLE_SERVICE_UUID);
    s_adv->setScanResponse(true);
    // Preferred connection interval 7.5-22.5 ms. iOS renegotiates to its own taste anyway,
    // but advertising a sane range shortens the connection setup. (Note the pair: every
    // Espressif BLE example carries a copy-paste bug that calls setMinPreferred twice.)
    s_adv->setMinPreferred(0x06);
    s_adv->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();

    s_active      = true;
    s_conn        = false;
    s_conn_handle = 0xFFFF;
    s_cmd         = LEAF_BLE_CMD_NONE;
    Serial.printf("[ble] advertising as \"%s\" (svc %s)\n", s_name, LEAF_BLE_SERVICE_UUID);
    return true;
}

const char* leaf_ble_name() { return s_name; }

void leaf_ble_set_status(const char* node, const char* fw, uint32_t boot_count,
                         uint32_t pir_wakes, uint32_t captures, uint32_t push_fails,
                         float vbat_v) {
    char buf[192];
    if (isnan(vbat_v)) {
        snprintf(buf, sizeof(buf),
                 "{\"node\":\"%s\",\"fw\":\"%s\",\"boot\":%u,\"pir\":%u,\"cap\":%u,\"fail\":%u}",
                 node ? node : "?", fw ? fw : "?", (unsigned)boot_count,
                 (unsigned)pir_wakes, (unsigned)captures, (unsigned)push_fails);
    } else {
        snprintf(buf, sizeof(buf),
                 "{\"node\":\"%s\",\"fw\":\"%s\",\"boot\":%u,\"pir\":%u,\"cap\":%u,\"fail\":%u,"
                 "\"vb\":%.2f}",
                 node ? node : "?", fw ? fw : "?", (unsigned)boot_count,
                 (unsigned)pir_wakes, (unsigned)captures, (unsigned)push_fails, vbat_v);
    }
    s_payload = String(buf);
    if (s_status) {
        s_status->setValue(s_payload);
        if (s_conn) s_status->notify();
    }
}

bool leaf_ble_connected() { return s_conn; }

LeafBleCmd leaf_ble_take_cmd() {
    LeafBleCmd c = s_cmd;
    s_cmd = LEAF_BLE_CMD_NONE;
    return c;
}

void leaf_ble_end(bool release_memory) {
    if (!s_active) return;
    s_stopping = true;          // gate the disconnect callback BEFORE anything is freed

    if (s_adv) s_adv->stop();

    // Disconnect any client OURSELVES and give the host a beat to finish the GAP teardown,
    // rather than letting deinit() do it mid-teardown. Both halves matter: without the
    // stopping flag the callback re-arms freed advertising, and without this ordered
    // disconnect the host is still unwinding a live link when its pools are released.
    if (s_server && s_conn && s_conn_handle != 0xFFFF) {
        Serial.printf("[ble] terminating link %u before deinit\n", (unsigned)s_conn_handle);
        s_server->disconnect(s_conn_handle);
    }
    // Wait for the host to actually report the link down. Deinit while a connection object
    // is still live is what corrupts the heap, and the terminate is asynchronous, so the
    // wait is the part that matters — not the call.
    for (int i = 0; i < 200 && s_conn; i++) delay(10);      // <= 2 s
    if (s_conn) Serial.println("[ble] WARNING: link still up at teardown");
    delay(150);   // let the host free the connection object before its pools go away

    // Drop our pointers before deinit so nothing can dereference them while the stack is
    // being dismantled.
    s_server = nullptr;
    s_status = nullptr;
    s_adv    = nullptr;

    // release_memory hands the controller's BSS to the general heap. Worth ~18 KB, and
    // ONE-WAY for the rest of the boot — see leaf_ble.h. Never pass true if this wake
    // still means to advertise again.
    BLEDevice::deinit(release_memory);

    s_active      = false;
    s_conn        = false;
    s_conn_handle = 0xFFFF;
    s_cmd         = LEAF_BLE_CMD_NONE;
    s_stopping    = false;
    Serial.printf("[ble] stack down (controller memory %s)\n",
                  release_memory ? "RELEASED - no restart this boot" : "kept, restartable");
}

bool leaf_ble_active() { return s_active; }

#endif  // LEAF_BLE
