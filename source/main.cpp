/* mbed Microcontroller Library
 * Copyright (c) 2006-2019 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include "ble/BLE.h"
#include "ble/Gap.h"

// FYI, issue "mbed deploy" at either the top directory (../mbed-os-example-ble)
// or the particular example directory to have all these peripherally
// dependent files such as the below, be included in the project.
#include "pretty_printer.h"
 
#include "mbed-trace/mbed_trace.h"

// Needed for Nuertey Odzeyem's ErrorCodesMap_t implementation scheme below.
#include <map>
#include <string>

const static char DEVICE_NAME[] = "NUCLEO-WB55RG";

using namespace std::literals::chrono_literals;

using ErrorCodesMap_t = std::map<ble_error_t, std::string>;
using IndexElement_t  = ErrorCodesMap_t::value_type;

inline static auto make_error_codes_map()
{
    ErrorCodesMap_t eMap;

    eMap.insert(IndexElement_t(BLE_ERROR_NONE, std::string("\"No error\"")));
    eMap.insert(IndexElement_t(BLE_ERROR_BUFFER_OVERFLOW, std::string("\"The requested action would cause a buffer overflow and has been aborted\"")));
    eMap.insert(IndexElement_t(BLE_ERROR_NOT_IMPLEMENTED, std::string("\"Requested a feature that isn't yet implemented or isn't supported by the target HW\"")));
    eMap.insert(IndexElement_t(BLE_ERROR_PARAM_OUT_OF_RANGE, std::string("\"One of the supplied parameters is outside the valid range\"")));
    eMap.insert(IndexElement_t(BLE_ERROR_INVALID_PARAM, std::string("\"One of the supplied parameters is invalid\"")));
    eMap.insert(IndexElement_t(BLE_STACK_BUSY, std::string("\"The stack is busy\"")));
    eMap.insert(IndexElement_t(BLE_ERROR_INVALID_STATE, std::string("\"Invalid state\"")));
    eMap.insert(IndexElement_t(BLE_ERROR_NO_MEM, std::string("\"Out of memory\"")));
    eMap.insert(IndexElement_t(BLE_ERROR_OPERATION_NOT_PERMITTED, std::string("\"The operation requested is not permitted\"")));
    eMap.insert(IndexElement_t(BLE_ERROR_INITIALIZATION_INCOMPLETE, std::string("\"The BLE subsystem has not completed its initialization\"")));
    eMap.insert(IndexElement_t(BLE_ERROR_ALREADY_INITIALIZED, std::string("\"The BLE system has already been initialized\"")));
    eMap.insert(IndexElement_t(BLE_ERROR_UNSPECIFIED, std::string("\"Unknown error\"")));
    eMap.insert(IndexElement_t(BLE_ERROR_INTERNAL_STACK_FAILURE, std::string("\"The platform-specific stack failed\"")));
    eMap.insert(IndexElement_t(BLE_ERROR_NOT_FOUND, std::string("\"Data not found or there is nothing to return\"")));

    return eMap;
}

static ErrorCodesMap_t gs_ErrorCodesMap = make_error_codes_map();

inline std::string ToString(const ble_error_t & key)
{
    std::string result;

    // Prevent the possibility of std::out_of_range exception if the container
    // does not have an error element with the specified key.
    auto iter = gs_ErrorCodesMap.find(key);
    if (iter != gs_ErrorCodesMap.end())
    {
        result = iter->second;
    }
    else
    {
        result = std::string("\"Warning! Code does not indicate an error and consequently does not exist in gs_ErrorCodesMap!\"");
    }

    return result;
}

// By default enough buffer space for 32 event Callbacks, i.e. 32*EVENTS_EVENT_SIZE
// Reduce this amount if the target device has severely limited RAM.
static events::EventQueue g_SharedEventQueue(16 * EVENTS_EVENT_SIZE);

// GAP
//
// GAP is an acronym for the Generic Access Profile, and it controls 
// connections and advertising in Bluetooth. GAP is what makes your device
// visible to the outside world, and determines how two devices can (or can't)
// interact with each other.
class BluetoothLowEnergyEncapsulation : ble::Gap::EventHandler
{
public:
    BluetoothLowEnergyEncapsulation(BLE &ble, events::EventQueue &event_queue) 
        : m_BluetoothLowEnergyStack(ble)
        , m_SharedEventQueue(event_queue)
        , m_TheBatteryLevel(50)
        // Deliberately omitting m_AdvertisingBuffer here so that the implicit
        // zero-initialization specified during declaration can kick in here.
        , m_AdvertisingDataBuilder(m_AdvertisingBuffer, ble::LEGACY_ADVERTISING_MAX_SIZE)
    {
    }

    void start()
    {
        /* mbed will call on_init_complete when when ble is ready */
        m_BluetoothLowEnergyStack.init(this, &BluetoothLowEnergyEncapsulation::on_init_complete);

        /* this will never return */
        m_SharedEventQueue.dispatch_forever();
    }

private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params)
    {
        if (params->error != BLE_ERROR_NONE)
        {
            printf("Error! BLE initialization failed: \
                [%d] -> %s\r\n", params->error, ToString(params->error).c_str());
            //print_error(params->error, "Ble initialization failed.");
            return;
        }

        print_mac_address();

        start_advertising();
    }

    void start_advertising()
    {
        // Advertising and Scan Response Data
        //
        // There are two ways to send advertising out with GAP. 
        // The Advertising Data payload and the Scan Response payload.
        //
        // Both payloads are identical and can contain up to 31 bytes of
        // data, but only the advertising data payload is mandatory, since
        // this is the payload that will be constantly transmitted out 
        // from the device to let central devices in range know that it 
        // exists. The scan response payload is an optional secondary 
        // payload that central devices can request, and allows device 
        // designers to fit a bit more information in the advertising
        // payload such a strings for a device name, etc.
        
        /* create advertising parameters and payload */
        ble::AdvertisingParameters advertisingParameters(
            /* you cannot connect to this device, you can only read its advertising data,
             * scannable means that the device has extra advertising data that the peer can receive if it
             * "scans" it which means it is using active scanning (it sends a scan request) */
            ble::advertising_type_t::SCANNABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(1000))
        );

        /* when advertising you can optionally add extra data that is only sent
         * if the central requests it by doing active scanning (sending scan requests),
         * in this example we set this payload first because we want to later reuse
         * the same m_AdvertisingDataBuilder builder for payload updates */
        const uint8_t _vendor_specific_data[4] = { 0xAD, 0xDE, 0xBE, 0xEF };
        m_AdvertisingDataBuilder.setManufacturerSpecificData(_vendor_specific_data);

        m_BluetoothLowEnergyStack.gap().setAdvertisingScanResponse(
            ble::LEGACY_ADVERTISING_HANDLE,
            m_AdvertisingDataBuilder.getAdvertisingData()
        );

        /* now we set the advertising payload that gets sent during 
         * advertising without any scan requests */
        m_AdvertisingDataBuilder.clear();
        m_AdvertisingDataBuilder.setFlags();
        m_AdvertisingDataBuilder.setName(DEVICE_NAME);

        /* we add the battery level as part of the payload so it's visible
         * to any device that scans, this part of the payload will be 
         * updated periodically without affecting the rest of the payload */
        m_AdvertisingDataBuilder.setServiceData(GattService::UUID_BATTERY_SERVICE, 
                                                {&m_TheBatteryLevel, 1});

        /* setup advertising */
        ble_error_t error = m_BluetoothLowEnergyStack.gap().setAdvertisingParameters(
                                ble::LEGACY_ADVERTISING_HANDLE,
                                advertisingParameters
                            );

        if (error)
        {
            printf("Error! _ble.gap().setAdvertisingParameters() failed: \
                [%d] -> %s\r\n", error, ToString(error).c_str());
            //print_error(error, "_ble.gap().setAdvertisingParameters() failed");
            return;
        }

        error = m_BluetoothLowEnergyStack.gap().setAdvertisingPayload(
                    ble::LEGACY_ADVERTISING_HANDLE,
                    m_AdvertisingDataBuilder.getAdvertisingData()
                );

        if (error)
        {
            printf("Error! _ble.gap().setAdvertisingPayload() failed: \
                [%d] -> %s\r\n", error, ToString(error).c_str());
            //print_error(error, "_ble.gap().setAdvertisingPayload() failed");
            return;
        }

        /* start advertising */
        error = m_BluetoothLowEnergyStack.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

        if (error)
        {
            printf("Error! _ble.gap().startAdvertising() failed: \
                [%d] -> %s\r\n", error, ToString(error).c_str());
            //print_error(error, "_ble.gap().startAdvertising() failed");
            return;
        }

        // Nuertey Odzeyem Note:
        //
        // Here, note that the EventQueue is not intrinsic nor necessary
        // for the operation of the BLE Advertisement Service. Advertisements
        // will always be ongoing, continuously. The EventQueue just helps
        // us simulate a battery charging and discharging so that its value
        // when changed can then be written into those continuous BLE
        // advertisements. Separation of the application business logic
        // and the BLE feature so to speak. To further clarify, the execution
        // context needed to schedule BLE events/callbacks, could have
        // been supplied by a thread.

        /* we simulate battery discharging by updating it every second */
        m_SharedEventQueue.call_every(1000ms, [this]()
                                              {
                                                  update_battery_level();
                                              }
        );
    }

    void update_battery_level()
    {
        if (m_TheBatteryLevel-- == 10)
        {
            m_TheBatteryLevel = 100;
        }

        // GATT
        //
        // GATT is an acronym for the Generic ATTribute Profile, and it defines
        // the way that two Bluetooth Low Energy devices transfer data back and
        // forth using concepts called Services and Characteristics. It makes use
        // of a generic data protocol called the Attribute Protocol (ATT), which
        // is used to store Services, Characteristics and related data in a
        // simple lookup table using 16-bit IDs for each entry in the table.
        // GATT comes into play once a dedicated connection is established between
        // two devices, meaning that you have already gone through the advertising
        // process governed by GAP.    
        
        // GATT Transactions
        //
        // An important concept to understand with GATT is the server/client relationship.
        // The peripheral is known as the GATT Server, which holds the ATT lookup data and
        // service and characteristic definitions, and the GATT Client (the phone/tablet), 
        // which sends requests to this server.
        // All transactions are started by the main device, the GATT Client, which receives
        // response from the secondary device, the GATT Server.

        // Broadcast Network Topology
        //
        // While most peripherals advertise themselves so that a connection can be established
        // and GATT services and characteristics can be used (which allows for much more data
        // to be exchanged and in both directions), there are situations where you only want to
        // advertise data.
        //
        // The main use case here is where you want a peripheral to send data to more than
        // one device at a time. This is only possible using the advertising packet since data
        // sent and received in connected mode can only be seen by those two connected
        // devices.
        //
        // By including a small amount of custom data in the 31 byte advertising or scan
        // response payloads, you can use a low cost Bluetooth Low Energy peripheral to sent
        // data one-way to any devices in listening range, as shown in the illustration below.
        // This is known as Broadcasting in Bluetooth Low Energy.
        //
        // This is the approach use by Apple's iBeacon, for example, which inserts a custom
        // payload in the main advertising packet, using the Manufacturer Specific Data field.
        // 
        // Once you establish a connection between your peripheral and a central device, the
        // advertising process will generally stop and you will typically no longer be able to send
        // advertising packets out anymore, and you will use GATT services and characteristics
        // to communicate in both directions.

        /* update the payload with the new value of the bettery level, 
         * the rest of the payload remains the same */
        ble_error_t error = m_AdvertisingDataBuilder.setServiceData(
                                      GattService::UUID_BATTERY_SERVICE, 
                                      make_Span(&m_TheBatteryLevel, 1));

        if (error)
        {
            printf("Error! _adv_data_builder.setServiceData() failed: \
                [%d] -> %s\r\n", error, ToString(error).c_str());
            //print_error(error, "_adv_data_builder.setServiceData() failed");
            return;
        }

        /* set the new payload, we don't need to stop advertising */
        error = m_BluetoothLowEnergyStack.gap().setAdvertisingPayload(
                    ble::LEGACY_ADVERTISING_HANDLE,
                    m_AdvertisingDataBuilder.getAdvertisingData()
                );

        if (error)
        {
            printf("Error! _ble.gap().setAdvertisingPayload() failed: \
                [%d] -> %s\r\n", error, ToString(error).c_str());
            //print_error(error, "_ble.gap().setAdvertisingPayload() failed");
            return;
        }
    }

private:
    // The Cordio Bluetooth stack only stores one single signing key. This key is then 
    // shared across all bonded devices. If a malicious device bonds with the Mbed OS 
    // application it then gains knowledge of the shared signing key of the Mbed OS device. 
    // The malicious device can then track the Mbed OS device whenever a signing write 
    // is issued from it. 
    // 
    // To overcome this privacy issue do not issue signed writes from the Mbed OS device.
    // A signed write occurs when the member function `write` of `GattClient` is called 
    // with its `cmd` argument set to `GATT_OP_SIGNED_WRITE_CMD`.
    // 
    // Instead of using signed writes, enable encryption on the connection. This is achieved
    // by calling the function `setLinkEncryption` of the `SecurityManager`. Set the encryption 
    // to at least `ENCRYPTED`. 
    BLE &                       m_BluetoothLowEnergyStack;
    
    events::EventQueue &        m_SharedEventQueue;
    uint8_t                     m_TheBatteryLevel; // The data to be broadcasted in the BLE advertisements.
    
    // Leverage C++11 member initializers to guarantee that the buffer 
    // is zeroed out upon construction before passing it on to the DataBuilder.
    uint8_t                     m_AdvertisingBuffer[ble::LEGACY_ADVERTISING_MAX_SIZE] = {};
    ble::AdvertisingDataBuilder m_AdvertisingDataBuilder;
};

/* Schedule processing of events from the BLE middleware in the global shared event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context)
{
    g_SharedEventQueue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

int main()
{
    printf("\r\n\r\n\"../mbed-os-example-ble/BLE_Advertising\" Application - Beginning... \r\n\r\n");
#ifdef MBED_MAJOR_VERSION
    printf("Mbed OS version: %d.%d.%d\n\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);
#endif
    printf("Built: %s, %s\n\n", __DATE__, __TIME__);
    mbed_trace_init();

    // "Got to a point where i am confident how to use a single service 
    // that comes with Mbed OS or i also can create a simple custom service…
    //
    // Now i just would like to use multiple services in my program logic.
    // What is the best practice to do that?
    // Should multiple services share the same GattServer object/event_queue/event_handler
    // or is it better to create own instances of these for each service?" 
    //
    // ANSWER:
    // "There is only one instance of BLE, so only one instance of gap, gatt, etc.
    //
    // Multiple services can run at the same time. We don’t have an example 
    // that uses two at the same time but there’s no trick to it. Just 
    // instantiate two (different ones) of them."
    //
    // https://forums.mbed.com/t/ble-proper-way-to-add-several-services/13628
    BLE &ble = BLE::Instance(); // Singleton
    ble.onEventsToProcess(schedule_ble_events);

    BluetoothLowEnergyEncapsulation demo(ble, g_SharedEventQueue);
    demo.start();

    // As per design, we will NEVER get to this statement. Great! Helps with debug...
    printf("\r\n\r\n\"../mbed-os-example-ble/BLE_Advertising\" - Exiting.\r\n\r\n");
    return 0;
}
