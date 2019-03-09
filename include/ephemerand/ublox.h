#pragma once

#include <memory>
#include <functional>
#include <iostream>
#include <string>
#include <variant>
#include <thread>

#include <hoytech/protected_queue.h>
#include <serial/serial.h>

#include "ephemerand/ublox_structures.h"



namespace ephemerand {




struct UbloxMessage_Version {
    std::string software_version;
    std::string hardware_version;
};

struct UbloxMessage_AlmanacMissing {
    uint32_t svprn;
};

struct UbloxMessage_AlmanacData {
    uint32_t svprn;
    uint32_t issue_week;
    std::string data;
};

struct UbloxMessage_Ack {
    uint64_t message_class;
    uint64_t message_id;
};

using UbloxMessage = std::variant<UbloxMessage_Version, UbloxMessage_AlmanacMissing, UbloxMessage_AlmanacData, UbloxMessage_Ack>;




class Ublox {
  public:
    Ublox(std::string device_, uint32_t baudrate_ = 115200)
        : device(device_), baudrate(baudrate_) {}

    void connect() {
        serial::Timeout timeout(100, 1000, 0, 1000, 0);
        serial = std::make_unique<serial::Serial>(device, baudrate, timeout);
        serial->flush();
    }

    void requestVersion() {
        pollMessage(MSG_CLASS_MON, MSG_ID_MON_VER);
    }

    void pollAlmanac() {
        pollMessage(MSG_CLASS_AID, MSG_ID_AID_ALM);
    }

    void pollSubframeBuffer() {
        pollMessage(MSG_CLASS_RXM, MSG_ID_RXM_SFRB);
    }

    void reset() {
        sendReset(0xFFFF, 0x02); // Coldstart, Controlled Software Reset - Only GPS
    }


    void run() {
        run_thread = std::thread([this]{
            requestVersion();

            while(1) {
                unsigned char result[16384];
                size_t bytes_read = serial->read(result, sizeof(result));
                if (!bytes_read) continue;

                buffer.append((char*)&result[0], bytes_read);

                while(processMessage()) {}
            }
        });
    }


    hoytech::protected_queue<UbloxMessage> output_queue;


  private:
    std::thread run_thread;
    std::string device;
    uint32_t baudrate;
    std::unique_ptr<serial::Serial> serial;
    std::string buffer;




    bool processMessage() {
        auto msg_start = buffer.find("\xb5\x62");
        if (msg_start == std::string::npos) {
            buffer.clear();
            return false;
        }

        buffer.erase(0, msg_start);

        if (buffer.size() < sizeof(ublox::UbloxHeader)) return false;

        auto *header = (struct ublox::UbloxHeader *)buffer.data();

        size_t total_size = sizeof(ublox::UbloxHeader) + header->payload_length + 2; // 2 checksum bytes

        if (buffer.size() < total_size) return false;

        std::string message = buffer.substr(sizeof(ublox::UbloxHeader), header->payload_length);
        uint8_t message_class = header->message_class;
        uint8_t message_id = header->message_id;

        buffer.erase(0, total_size);
        handleMessage(message_class, message_id, message);
        return true;
    }


    void handleMessage(uint8_t message_class, uint8_t message_id, std::string &message) {
        if (message_class == MSG_CLASS_MON && message_id == MSG_ID_MON_VER) {
            std::string software_version((char*)(message.data()));
            std::string hardware_version;
            if (message.size() > 30) hardware_version.append((char*)(message.data() + 30));
            output_queue.push_move(UbloxMessage_Version{software_version, hardware_version});
        } else if (message_class == MSG_CLASS_AID && message_id == MSG_ID_AID_ALM) {
            if (message.size() == sizeof(ublox::AlmanacNoPayload)) {
                auto *alm = (ublox::AlmanacNoPayload *)message.data();
                output_queue.push_move(UbloxMessage_AlmanacMissing{alm->svprn});
            } else if (message.size() == sizeof(ublox::AlmanacWithPayload)) {
                auto *alm = (ublox::AlmanacWithPayload *)message.data();
                output_queue.push_move(UbloxMessage_AlmanacData{alm->svprn, alm->issue_week, message.substr(8)});
            } else {
                std::cerr << "Almanac message with unrecognized size (" << message.size() << "), skipping" << std::endl;
                return;
            }
        } else if (message_class == MSG_CLASS_ACK && message_id == MSG_ID_ACK_ACK) {
            auto *cfg = (ublox::CfgMsgPayload *)message.data();
            output_queue.push_move(UbloxMessage_Ack{cfg->message_class, cfg->message_id});
        } else {
            std::cerr << "Unknown ublox message type: " << int(message_class) << " / " << int(message_id) << std::endl;
        }
    }



    // Following functions adapted from https://github.com/GAVLab/ublox.git

    // Poll Message used to request for all SV
    bool pollMessage(uint8_t class_id, uint8_t msg_id) {
        try {
            uint8_t message[8];

            message[0]=UBX_SYNC_BYTE_1;        // sync 1
            message[1]=UBX_SYNC_BYTE_2;        // sync 2
            message[2]=class_id;
            message[3]=msg_id;
            message[4]=0;           // length 1
            message[5]=0;           // length 2
            message[6]=0;           // checksum 1
            message[7]=0;           // checksum 2

            uint8_t* msg_ptr = (uint8_t*) &message;

            calculateCheckSum(msg_ptr + 2, 4, msg_ptr + 6);

            if ((serial!=NULL)&&(serial->isOpen())) {
                size_t bytes_written = serial->write(message, 8);
                return bytes_written == 8;
            } else {
                std::cerr << "Unable to send version message since serial port not open" << std::endl;
                return false;
            }
        } catch (std::exception &e) {
            std::cerr << "Error sending ublox poll message: " << e.what() << std::endl;
            return false;
        }
    }

    bool sendReset(uint16_t nav_bbr_mask, uint8_t reset_mode) {
        try {
            ublox::CfgRst message;

            message.header.sync1 = UBX_SYNC_BYTE_1;
            message.header.sync2 = UBX_SYNC_BYTE_2;
            message.header.message_class = MSG_CLASS_CFG;
            message.header.message_id = MSG_ID_CFG_RST;
            message.header.payload_length = 4;

            message.nav_bbr_mask = nav_bbr_mask;    //X2-Bitfield?
            // Startup Modes
            // Hotstart 0x000
            // Warmstart 0x0001
            // Coldstart 0xFFFF
            message.reset_mode = reset_mode;
            // Reset Modes:
            // Hardware Reset 0x00
            // Controlled Software Reset 0x01
            // Controlled Software Reset - Only GPS 0x02
            // Hardware Reset After Shutdown 0x04
            // Controlled GPS Stop 0x08
            // Controlled GPS Start 0x09

            //message.reserved = 0;

            unsigned char* msg_ptr = (unsigned char*) &message;
            calculateCheckSum(msg_ptr + 2, 8, message.checksum);

            if ((serial!=NULL)&&(serial->isOpen())) {
                return serial->write(msg_ptr, sizeof(message)) == sizeof(message);
            } else {
                std::cerr << "Unable to send reset command. Serial port not open." << std::endl;
                return false;
            }
        } catch (std::exception &e) {
            std::stringstream output;
            std::cerr << "Error resetting ublox: " << e.what() << std::endl;
            return false;
        }
    }

    bool ConfigureMessageRate(uint8_t class_id, uint8_t msg_id, uint8_t rate) {
        try {
            ublox::CfgMsgRate message;
            message.header.sync1 = UBX_SYNC_BYTE_1;
            message.header.sync2 = UBX_SYNC_BYTE_2;
            message.header.message_class = MSG_CLASS_CFG;
            message.header.message_id = MSG_ID_CFG_MSG;
            message.header.payload_length = 3;

            message.message_class = class_id;
            message.message_id = msg_id;
            message.rate = rate;

            unsigned char* msg_ptr = (unsigned char*) &message;
            calculateCheckSum(msg_ptr + 2, 7, message.checksum);

            return serial->write(msg_ptr, sizeof(message)) == sizeof(message);
        } catch (std::exception &e) {
            std::cerr << "Error configuring ublox message rate: " << e.what() << std::endl;
            return false;
        }
    }

    void calculateCheckSum(uint8_t* in, size_t length, uint8_t* out) {
        uint8_t a = 0;
        uint8_t b = 0;

        for (size_t i = 0; i < length; i++) {
            a = a + in[i];
            b = b + a;
        }

        out[0] = (a & 0xFF);
        out[1] = (b & 0xFF);
    }
};


}
