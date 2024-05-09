/* Copyright (c) 2016, Oleksiy Kebkal <lesha@evologics.de>
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met: 
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 * 3. The name of the author may not be used to endorse or promote products 
 *    derived from this software without specific prior written permission. 
 * 
 * Alternatively, this software may be distributed under the terms of the 
 * GNU General Public License ("GPL") version 2 as published by the Free 
 * Software Foundation. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 */
#ifndef DMAC_PARSER_H
#define DMAC_PARSER_H

#include <math.h> 

#include <iostream>
#include <sstream>

#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>

#include "dmac/DMACPayload.h"
#include "dmac/DMACRaw.h"
#include "dmac/DMACAsync.h"
#include "dmac/DMACSync.h"
#include "dmac/DMACClock.h"
#include "dmac/mUSBLFix.h"
#include "diagnostic_msgs/KeyValue.h"

#include "comm_middlemen.h"
#include "config.h"
#include "initializer.h"
#include "aparser.h"

using boost::asio::ip::tcp;
using dmac::DMACPayload;
using dmac::DMACRaw;
using dmac::DMACAsync;
using dmac::DMACSync;
using dmac::DMACClock;
using dmac::mUSBLFix;

namespace dmac
{

class parser : public dmac::abstract_parser
{
  public:
  parser(boost::asio::io_service& io_service, dmac::config &config, dmac::comm_middlemen *comm)
        :  filter_(DMAC_AT),
        config_(config),
        io_service_(io_service),
        answer_timer_(io_service),
        state_(DMAC_UNDEF),
        mode_(DMAC_DATA_MODE),
        waitsync_(WAITSYNC_NO),
        request_(""),
        request_parameters_(""),
        eol_("\n"),
        ext_networking_(false),
        pid_(0),
        ini_(this, &config)
    {
        pub_recv_ = nh_.advertise<DMACPayload>(config.nodeName() + "/recv", 100);
        pub_async_ = nh_.advertise<DMACAsync>(config.nodeName() + "/async", 100);
        pub_clock_ = nh_.advertise<DMACClock>(config.nodeName() + "/clock", 100);
        pub_raw_ = nh_.advertise<DMACRaw>(config.nodeName() + "/raw", 100);
        pub_usblfix_ = nh_.advertise<mUSBLFix>(config.nodeName() + "/measurement/usbl_fix", 100);
        pub_sync_ = nh_.advertise<DMACSync>(config.nodeName() + "/sync", 100);
        sub_sync_ = nh_.subscribe(config.nodeName() + "/sync", 100, &parser::syncCallback, this);
        sub_send_ = nh_.subscribe(config.nodeName() + "/send", 100, &parser::sendCallback, this);
        comm_ = comm;
    }

    void ctrl(parser_state_ctrl ctrl, std::string value)
    {
        switch (ctrl) {
        case EOL:
            eol_ = value;
            break;
        default:
            ROS_ERROR_STREAM("Unsupported ctrl: " << ctrl);
        }
    }
    
    void ctrl(parser_state_ctrl ctrl, int value)
    {
        switch (ctrl) {
        case WAITSYNC: {
            waitsync_ = (dmac_waitsync_status)value;
            break;
        }
        case MODE: {
            mode_ = (dmac_filter_mode)value;
            break;
        }
        case FILTER: {
            filter_ = (dmac_filter_type)value;
            break;
        }
        default:
            ROS_ERROR_STREAM("Unsupported ctrl: " << ctrl);
        }
    }
    
    void connected(void)
    {
        ini_.connected();
    }

    void disconnected(void)
    {
        ctrl(WAITSYNC, WAITSYNC_NO);
        ctrl(MODE, DMAC_DATA_MODE);
        ctrl(FILTER, DMAC_AT);
        ini_.disconnected();
    }
    
    void to_term(std::vector<uint8_t> chunk, std::size_t len)
    {
        std::string schunk;
        std::vector<uint8_t>::iterator it = chunk.begin();
        for (int cnt = 0; cnt < len; it++, cnt++) {
            schunk.insert(schunk.end(), *it);
        }
        ROS_INFO_STREAM("Parsing new data(" << mode_ << "): " << schunk);
        ROS_INFO_STREAM("waitsync_ : " << waitsync_);
        ROS_INFO_STREAM("more_: " << more_);
        more_ += schunk;
        publishRaw(schunk);
        
        size_t before, after;
        do {
            before = more_.length();
            if (filter_ == DMAC_AT && mode_ == DMAC_DATA_MODE) {
                to_term_at();
            } else {
                to_term_net();
            }
            after = more_.length();
        } while (after > 0 && after != before);
    }
    
    void sendCallback(const dmac::DMACPayload::ConstPtr& msg)
    {
        if (!ini_.ready()) {
            ROS_ERROR_STREAM("Driver not yet initialized");
            publishSync("ERROR NOT INITIALIZED", WAITSYNC_NO);
            return;
        }
        if (waitsync_ == WAITSYNC_NO) {
            request_parameters_ = "";
            waitsync_ = WAITSYNC_SINGLELINE;
            std::ostringstream stream;
            std::string prefix = ((filter_ == DMAC_AT && mode_ == DMAC_DATA_MODE) ? "+++" : "");
            switch (msg->type) {
            case DMACPayload::DMAC_BURST: {
                /* AT*SEND,<len>,<addr>,<payload>\r\n */
                request_ = "*SEND";
                stream << prefix << "AT" << request_ << "," << msg->payload.length() <<
                    "," << (int)msg->destination_address << "," << msg->payload << eol_;
                std::string msg = stream.str();
                sendSync(msg);
                break;
            }
            case DMACPayload::DMAC_IM: {
                /* AT*SENDIM,<len>,<addr>,<ack>,<payload>\r\n */
                request_ = "*SENDIM";
                std::string ack = (msg->ack ? "ack" : (msg->force ? "force" : "noack"));
                stream << prefix << "AT" << request_ << "," << msg->payload.length() <<
                    "," << (int)msg->destination_address << "," << ack << "," << msg->payload << eol_;
                std::string msg = stream.str();
                sendSync(msg);
                break;
            }
            case DMACPayload::DMAC_IMS: {
                /* AT*SENDIMS,<len>,<addr>,[<timestamp>],<payload>\r\n */
                request_ = "*SENDIMS";
                stream << prefix << "AT" << request_ << "," << msg->payload.length() <<
                    "," << (int)msg->destination_address << "," <<
                    (msg->timestamp_undefined ? std::string("") : boost::lexical_cast<std::string>(msg->timestamp)) <<
                    "," << msg->payload << eol_;
                std::string msg = stream.str();
                sendSync(msg);
                break;
            }
            case DMACPayload::DMAC_PBM: {
                /* AT*SENDPBM,<len>,<addr>,<payload>\r\n */
                request_ = "*SENDPBM";
                stream << prefix << "AT" << request_ << "," << msg->payload.length() <<
                    "," << (int)msg->destination_address << "," << msg->payload << eol_;
                std::string msg = stream.str();
                sendSync(msg);
                break;
            }
            case DMACPayload::DMAC_ECLK: {
                /* AT?ECLK */
                stream << msg->payload << eol_;
                std::string msg = stream.str();
                sendSync(msg);
                break;
            }
            default:
                ROS_ERROR_STREAM("" << __func__ << ": unsupported message type: " << msg->type);
                break;
            }
        } else {
            publishSync("ERROR SEQUENCE ERROR", waitsync_);
            ROS_ERROR_STREAM("" << __func__ << ": sequence error");
        }
    }

    void syncCallback(const dmac::DMACSync::ConstPtr& msg, bool privilege)
    { /* privilege true for initialiser */
        if (!msg->report.empty()) {
            /* ignore published by oursleves modem response */
            return;
        }
        if (waitsync_ == WAITSYNC_NO) {
            if (msg->command == "$") {
                ROS_ERROR_STREAM("TODO: " << __func__ << "(" << __LINE__ << ")");
            }
            else
            {
                waitsync_ = WAITSYNC_SINGLELINE;
                if (msg->command == "?ZSL" ||
                    msg->command == "?P" ||
                    msg->command == "&V" ||
                    (msg->command == "?S" && filter_ == DMAC_NET)) {
                    waitsync_ = WAITSYNC_MULTILINE;
                } else if (msg->command == "?NOISE") {
                    waitsync_ = WAITSYNC_BINARY;
                } else if (msg->command == "O") {
                    waitsync_ = WAITSYNC_NO;
                    answer_timer_.cancel();
                }
                request_ = boost::to_upper_copy<std::string>(msg->command);
                std::string prefix = ((filter_ == DMAC_AT && mode_ == DMAC_DATA_MODE) ? "+++" : "");
                request_parameters_ = msg->parameters;
                std::string telegram = prefix + "AT" + request_ + msg->parameters + eol_;
                sendSync(telegram, privilege);
            }
        } else {
            ROS_ERROR_STREAM("" << __func__ << ": sequence error");
            publishSync("ERROR SEQUENCE ERROR", waitsync_);
        }
    }

    void syncCallback(const dmac::DMACSync::ConstPtr& msg)
    {
        syncCallback(msg, false);
    }
    
    void handle_answer_timeout(const boost::system::error_code& error) {
        if (error == boost::asio::error::operation_aborted) {
            return;
        }
        ROS_ERROR_STREAM("Answer timeout: waitsync: " << waitsync_);
        publishSync("ERROR ANSWER TIMEOUT", WAITSYNC_NO);
    }
    
  private:
    boost::asio::io_service& io_service_;
    dmac::comm_middlemen *comm_;
    dmac::config config_;
    dmac::initializer ini_;
    /* parser state */
    ros::NodeHandle nh_;
    dmac_filter_type filter_;
    dmac_filter_mode mode_;
    dmac_filter_state state_;
    dmac_waitsync_status waitsync_;
    std::string request_;
    std::string request_parameters_;
    std::string eol_;
    bool ext_networking_;
    int pid_;
    std::string more_;
    boost::asio::deadline_timer answer_timer_;

    ros::Publisher pub_recv_;
    ros::Publisher pub_raw_;
    ros::Publisher pub_async_;
    ros::Publisher pub_clock_;
    ros::Publisher pub_usblfix_;
    ros::Publisher pub_sync_;
    ros::Subscriber sub_sync_;
    ros::Subscriber sub_send_;

    void sendSync(std::string &message, bool privilege)
    {
        if (privilege || ini_.ready()) {
            comm_->send(message);
            publishRaw(message);
            
            answer_timer_.cancel();
            answer_timer_.expires_from_now(boost::posix_time::milliseconds(1000));
            answer_timer_.async_wait(boost::bind(&parser::handle_answer_timeout, this,
                                                 boost::asio::placeholders::error));
        } else {
            ROS_ERROR_STREAM("Driver not yet initialized");
            publishSync("ERROR NOT INITIALIZED", WAITSYNC_NO);
        }
    }
    
    void sendSync(std::string &message)
    {
        sendSync(message, false);
    }

    void publishRaw(std::string raw)
    {
        DMACRaw raw_msg;
        raw_msg.stamp = ros::Time::now();
        raw_msg.command = raw;

        pub_raw_.publish(raw_msg);
    }
    
    void publishSync(std::string report, dmac_waitsync_status waitsync)
    {
        waitsync_ = waitsync;
        
        DMACSync sync_msg;
        sync_msg.header.stamp = ros::Time::now();
        sync_msg.command = request_;
        sync_msg.parameters = request_parameters_;
        sync_msg.report = report;
        if (ini_.state() == dmac::FINAL) {
            pub_sync_.publish(sync_msg);
        } else {
            ini_.sync(sync_msg);
            if (sync_msg.report == "ERROR NOT INITIALIZED") {
                pub_sync_.publish(sync_msg);
            }
        }
    }

    void publishUSBLFix(mUSBLFix &fix)
    {
        pub_usblfix_.publish(fix);
    }
    
    void publishRecv(DMACPayload &rcv)
    {
        if (ini_.state() == dmac::FINAL) {
            pub_recv_.publish(rcv);
        } else {
            ini_.raw(rcv);
        }
    }

    void publishAsync(DMACAsync &async)
    {
        pub_async_.publish(async);
    }

    void publishClock(DMACClock &clk)
    {
        pub_clock_.publish(clk);
    }
    
    void to_term_at()
    { /* bes_split */
        static const boost::regex bes_regex("((.*?)(\\+{3}AT.*?):(\\d+):)(.*)");
        boost::smatch bes_matches;
        if (boost::regex_match(more_, bes_matches, bes_regex))
        {
            size_t len = boost::lexical_cast<size_t>(bes_matches[4].str().data());
            if (len + 2 <= bes_matches[5].length())
            {
                boost::regex body_regex("^(.{" + boost::lexical_cast<std::string>(len) + "})\r\n(.*)");
                boost::smatch body_matches;
                std::string input = bes_matches[5].str();
                if (boost::regex_match(input, body_matches, body_regex))
                {
                    std::string raw = bes_matches[2];
                    recv_std_extract(raw);
                    std::string body = body_matches[1] + "\r\n";
                    std::string req = bes_matches[3].str().substr(5);
                    if ((waitsync_ == WAITSYNC_NO) ||
                        req.empty() ||
                        (req == request_))
                    {
                        more_ = body;
                        to_term_net();
                        if (!more_.empty()) {
                            ROS_ERROR_STREAM("BES parse error: " << more_);
                        }
                    }
                    else
                    {
                        ROS_ERROR_STREAM("BES unexpected sync message: " << body);
                    }
                    more_ = body_matches[2];
                }
                else
                {
                    std::string raw = bes_matches[1];
                    recv_std_extract(raw);
                    more_.erase(0, raw.length());
                }
            }
            else
            {
                ROS_WARN_STREAM("need more data: " << more_.data());
            }
        }
        else
        {
            static const boost::regex maybe_bes_regex("(.*?[^+]*)(\\+{3}(AT.*?:\\d*|AT[^:]{0,10}:?|AT?|A?)|\\+{0,2})$");
            boost::smatch maybe_bes_matches;
            if (boost::regex_match(more_, maybe_bes_matches, maybe_bes_regex))
            {
                if (!maybe_bes_matches[1].str().empty()) {
                    std::string raw = maybe_bes_matches[1];
                    recv_std_extract(raw);
                    more_.erase(0, raw.length());
                }
                if (!maybe_bes_matches[2].str().empty()) {
                    ROS_WARN_STREAM("need more data: " << more_.data());
                }
            }
            else
            {
                ROS_ERROR_STREAM("unexpected error parsing: " << more_);
            }
        }
    }
    
    void to_term_net()
    { /* answer_split */
        
        // to test if we got a message response to the ECLK command issued
        static const boost::regex clk_regex("^([^.]*)\\.([^,]*),([^,]*),([^,]*),([^,]*),([^.]*)\\.([^,]*)");
        boost::smatch clk_matches;

        static const boost::regex eol_regex("\r\n");
        
        if (boost::regex_search(more_, eol_regex))
        {
            static const boost::regex rcv_regex("^(RECV(|PBM|IM|IMS),)(p(\\d+),(\\d+)|(\\d+))(,.*)");
            boost::smatch rcv_matches;
            if (boost::regex_match(more_, rcv_matches, rcv_regex))
            {
                int len, pid;
                /* if rcv_matches[3] starts with p - it is new syntax */
                if (rcv_matches[3].str()[0] == 'p')
                {
                    pid = boost::lexical_cast<int>(rcv_matches[4].str().data());
                    len = boost::lexical_cast<int>(rcv_matches[5].str().data());
                }
                else
                {
                    pid = pid_;
                    len = boost::lexical_cast<int>(rcv_matches[6].str().data());
                }
                rcv_extract(rcv_matches[1].str(), pid, len, rcv_matches[7].str());
            }
            else if (boost::regex_match(more_, clk_matches, clk_regex)) 
            {
                static const boost::regex pattern("([^\\r\\n]+)"); // Define the regular expression pattern
                boost::smatch matches; // Search for matches

                if (boost::regex_search(more_, matches, pattern)) {
                    // properly process the eclk output and publish to specific topic with DMACClock msg type
                    eclk(matches[1]);
                }
            }
            else
            {
                static const boost::regex async_regex(
                    "^(RECVSTART|RECVEND,|RECVFAILED,|SEND[^,]*,|BITRATE,|RADDR,|SRCLEVEL,|PHYON|PHYOFF|USBL[^,]*,"
                    "|DROPCNT|DELIVERED|FAILED|EXPIRED|CANCELED|ECLK,)(.*?)\r\n(.*)");
                boost::smatch async_matches;
                /* 1 - asyn keyword, 2 - async parameters, 3 - rest */
                if (boost::regex_match(more_, async_matches, async_regex)) {
                    async_extract(async_matches[1].str(), async_matches[2].str());
                    more_ = async_matches[3].str();
                } else {
                    static const boost::regex error_regex("^((ERROR|BUSY) (.*?))\r\n(.*)");
                    boost::smatch error_matches;
                    if (boost::regex_match(more_, error_matches, error_regex)) {
                        std::string report = error_matches[1];
                        more_.erase(0, error_matches[1].str().length() + 2);
                        answer_timer_.cancel();
                        publishSync(report, WAITSYNC_NO);
                    } else {
                        /* the rest is sync answer, may be not yet full one */
                        switch (waitsync_) {
                        case WAITSYNC_NO: {
                            size_t pos = more_.find("\r\n");
                            ROS_ERROR_STREAM("Unexpected sync message: " << more_.substr(0, pos));
                            more_.erase(0, pos + 2);
                            break;
                        }
                        case WAITSYNC_SINGLELINE:
                        case WAITSYNC_MULTILINE: {
                            #define DMAC_EOT (waitsync_ == WAITSYNC_SINGLELINE ? "\r\n" : "\n\r\n")
                            #define DMAC_EOT_LEN (waitsync_ == WAITSYNC_SINGLELINE ? 2 : 3)
                            size_t pos = more_.find(DMAC_EOT);
                            if (pos != std::string::npos) {
                                std::string report = more_.substr(0, pos + DMAC_EOT_LEN);
                                more_.erase(0, pos + DMAC_EOT_LEN);
                                answer_timer_.cancel();
                                publishSync(report, WAITSYNC_NO);
                            } else {
                                ROS_WARN_STREAM("need more data: " << more_.data());
                            }
                            break;
                        }
                        case WAITSYNC_BINARY:
                            ROS_ERROR_STREAM("TODO: " << __func__ << "(" << __LINE__ << ")");
                            break;
                        }
                    }
                }
            }
        }
        else
        {
            ROS_WARN_STREAM("need more data: " << more_.data());
        }
    }

    void async_extract(std::string async, std::string parameters)
    {
        ROS_INFO_STREAM("processing " << async);
        if (async == "RECVSTART") {
            recvstart(parameters);
        } else if (async == "RECVEND,") {
            recvend(parameters);
        } else if (async == "RECVFAILED,") {
            recvfailed(parameters);
        } else if (async == "PHYOFF") {
            phyoff(parameters);
        } else if (async == "PHYON") {
            phyon(parameters);
        } else if (async == "SENDSTART,") {
            sendstart(parameters);
        } else if (async == "SENDEND,") {
            sendend(parameters);
        } else if (async == "USBLLONG,") {
            usbllong(parameters);
        } else if (async == "USBLANGLES,") {
            usblangles(parameters);
        } else if (async == "USBLPHYD,") {
            usblphyd(parameters);
        } else if (async == "USBLPHYP,") {
            usblphyp(parameters);
        } else if (async == "BITRATE,") {
            bitrate(parameters);
        } else if (async == "RADDR,") {
            raddr(parameters);
        } else if (async == "DELIVERED") {
            delivered(parameters);
        } else if (async == "FAILED") {
            failed(parameters);
        } else if (async == "CANCELED") {
            canceled(parameters);
        } else if (async == "EXPIRED") {
            expired(parameters);
        } else if (async == "SRCLEVEL,") {
            srclevel(parameters);
        } else if (async == "ECLK,") {
            eclk(parameters);
        } else if (async == "DROPCNT,") {
            dropcnt(parameters);
        } else {
            ROS_WARN_STREAM("unsupported async: " << async);
        }
    }

#define KEYVALUE_HELPER(msg__, key__, l__) do {     \
        kv.key = (key__);                           \
        kv.value = *(l__.begin());                  \
        msg__.map.push_back(kv);                    \
        l__.pop_front();                            \
    } while (0)
    
        
    void recvstart(std::string parameters)
    {
        DMACAsync async_msg;
        async_msg.header.stamp = ros::Time::now();
        async_msg.async = "recvstart";
        publishAsync(async_msg);
    }
    
    void recvend(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        static const boost::regex comma(",");
        std::list<std::string> l;
        boost::regex_split(std::back_inserter(l), parameters, comma);
        if (l.size() == 4)
        {
            DMACAsync async_msg;
            async_msg.header.stamp = ros::Time::now();
            async_msg.async = "recvend";
            KEYVALUE_HELPER(async_msg, "timestamp", l);
            KEYVALUE_HELPER(async_msg, "duration", l);
            KEYVALUE_HELPER(async_msg, "rssi", l);
            KEYVALUE_HELPER(async_msg, "integrity", l);
            publishAsync(async_msg);
        }
        else
        {
            ROS_ERROR_STREAM("" << __func__ << ": expected parameter count " << l.size() << " is not equal to 4.");
        }
    }
    
    void recvfailed(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        static const boost::regex comma(",");
        std::list<std::string> l;
        boost::regex_split(std::back_inserter(l), parameters, comma);
        if (l.size() == 3)
        {
            DMACAsync async_msg;
            async_msg.header.stamp = ros::Time::now();
            async_msg.async = "recvfailed";
            KEYVALUE_HELPER(async_msg, "relative_velocity", l);
            KEYVALUE_HELPER(async_msg, "rssi", l);
            KEYVALUE_HELPER(async_msg, "integrity", l);
            publishAsync(async_msg);
        }
        else
        {
            ROS_ERROR_STREAM("" << __func__ << ": expected parameter count " << l.size() << " is not equal to 3.");
        }
    }
    
    void phyoff(std::string parameters)
    {
        DMACAsync async_msg;
        async_msg.header.stamp = ros::Time::now();
        async_msg.async = "phyoff";
        publishAsync(async_msg);
    }
    
    void phyon(std::string parameters)
    {
        DMACAsync async_msg;
        async_msg.header.stamp = ros::Time::now();
        async_msg.async = "phyon";
        publishAsync(async_msg);
    }
    
    void sendstart(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        static const boost::regex comma(",");
        std::list<std::string> l;
        boost::regex_split(std::back_inserter(l), parameters, comma);
        if (l.size() == 4)
        {
            DMACAsync async_msg;
            async_msg.header.stamp = ros::Time::now();
            async_msg.async = "sendstart";
            KEYVALUE_HELPER(async_msg, "destination_address", l);
            KEYVALUE_HELPER(async_msg, "type", l);
            KEYVALUE_HELPER(async_msg, "duration", l);
            KEYVALUE_HELPER(async_msg, "delay", l);
            publishAsync(async_msg);
        }
        else
        {
            ROS_ERROR_STREAM("" << __func__ << ": expected parameter count " << l.size() << " is not equal to 4.");
        }
    }
    
    void sendend(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        static const boost::regex comma(",");
        std::list<std::string> l;
        boost::regex_split(std::back_inserter(l), parameters, comma);
        if (l.size() == 4)
        {
            DMACAsync async_msg;
            async_msg.header.stamp = ros::Time::now();
            async_msg.async = "sendend";
            KEYVALUE_HELPER(async_msg, "destination_address", l);
            KEYVALUE_HELPER(async_msg, "type", l);
            KEYVALUE_HELPER(async_msg, "timestamp", l);
            KEYVALUE_HELPER(async_msg, "duration", l);
            publishAsync(async_msg);
        }
        else
        {
            ROS_ERROR_STREAM("" << __func__ << ": expected parameter count " << l.size() << " is not equal to 4.");
        }
    }
    
    void usbllong(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        static const boost::regex comma(",");
        std::list<std::string> l;
        boost::regex_split(std::back_inserter(l), parameters, comma);
        if (l.size() == 16)
        {
            mUSBLFix fix_msg;
            fix_msg.header.stamp = ros::Time::now();
            fix_msg.header.frame_id = "usbl";
            fix_msg.type = mUSBLFix::FULL_FIX;
            std::list<std::string>::iterator it = l.begin();
            double telegram_time = boost::lexical_cast<double>(*it++);
            double measurement_time = boost::lexical_cast<double>(*it++);
            std::string src = *it++;
            fix_msg.source_id = boost::lexical_cast<int>(src.c_str());
            fix_msg.source_name = src;
            
            double x = boost::lexical_cast<double>(*it++);
            double y = boost::lexical_cast<double>(*it++);
            double z = boost::lexical_cast<double>(*it++);

            fix_msg.bearing_raw = atan2(y,x);
            fix_msg.elevation_raw = atan2(z, sqrt(x*x + y*y));

            x = fix_msg.relative_position.x = boost::lexical_cast<double>(*it++);
            y = fix_msg.relative_position.y = boost::lexical_cast<double>(*it++);
            z = fix_msg.relative_position.z = boost::lexical_cast<double>(*it++);

            fix_msg.bearing = atan2(y,x);
            fix_msg.elevation = atan2(z, sqrt(x*x + y*y));

            it++; it++; it++;

            fix_msg.sound_speed = 1500;
            fix_msg.range = boost::lexical_cast<double>(*it++) * fix_msg.sound_speed * 1e-6;

            publishUSBLFix(fix_msg);
        }
        else
        {
            ROS_ERROR_STREAM("" << __func__ << ": expected parameter count " << l.size() << " is not equal to 16.");
        }
    }
    
    void usblangles(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        static const boost::regex comma(",");
        std::list<std::string> l;
        boost::regex_split(std::back_inserter(l), parameters, comma);
        if (l.size() == 13)
        {
            mUSBLFix fix_msg;
            fix_msg.header.stamp = ros::Time::now();
            fix_msg.header.frame_id = "usbl";
            fix_msg.type = mUSBLFix::AZIMUTH_ONLY;
            std::list<std::string>::iterator it = l.begin();
            double telegram_time = boost::lexical_cast<double>(*it++);
            double measurement_time = boost::lexical_cast<double>(*it++);
            std::string src = *it++;
            fix_msg.source_id = boost::lexical_cast<int>(src.c_str());
            fix_msg.source_name = src;

            double lbearing = boost::lexical_cast<double>(*it++);
            double lelevation = boost::lexical_cast<double>(*it++);

            double bearing = boost::lexical_cast<double>(*it++);
            double elevation = boost::lexical_cast<double>(*it++);

            *it++;
            *it++;
            *it++;
            *it++;
            *it++;

            double accuracy =  boost::lexical_cast<double>(*it);
            //ROS_ERROR_STREAM("" << __func__ << ": accuracy = " << accuracy);

            fix_msg.bearing_raw = lbearing;
            fix_msg.elevation_raw = lelevation;
            
            if (config_.hasAHRS()) {
                fix_msg.bearing = bearing;
                fix_msg.elevation = elevation;
            } else {
                fix_msg.bearing = 0;
                fix_msg.elevation = 0;
            }
            fix_msg.sound_speed = 1500;

            if (accuracy >= 0)
                publishUSBLFix(fix_msg);
        }
        else
        {
            ROS_ERROR_STREAM("" << __func__ << ": expected parameter count " << l.size() << " is not equal to 13.");
        }
    }

    void usblphyd(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        static const boost::regex comma(",");
        std::list<std::string> l;
        boost::regex_split(std::back_inserter(l), parameters, comma);
        if (l.size() == 12)
        {
            DMACAsync async_msg;
            async_msg.header.stamp = ros::Time::now();
            async_msg.async = "usblphyd";
            KEYVALUE_HELPER(async_msg, "telegram_time", l);
            KEYVALUE_HELPER(async_msg, "measurement_time", l);
            KEYVALUE_HELPER(async_msg, "source_address", l);
            KEYVALUE_HELPER(async_msg, "type", l);
            KEYVALUE_HELPER(async_msg, "delay 1-5", l);
            KEYVALUE_HELPER(async_msg, "delay 2-5", l);
            KEYVALUE_HELPER(async_msg, "delay 3-5", l);
            KEYVALUE_HELPER(async_msg, "delay 4-5", l);
            KEYVALUE_HELPER(async_msg, "delay 1-2", l);
            KEYVALUE_HELPER(async_msg, "delay 4-1", l);
            KEYVALUE_HELPER(async_msg, "delay 3-2", l);
            KEYVALUE_HELPER(async_msg, "delay 3-4", l);
            publishAsync(async_msg);
        }
        else
        {
            ROS_ERROR_STREAM("" << __func__ << ": expected parameter count " << l.size() << " is not equal to 12.");
        }
    }
    
    void usblphyp(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        static const boost::regex comma(",");
        std::list<std::string> l;
        boost::regex_split(std::back_inserter(l), parameters, comma);
        ROS_INFO_STREAM("phyp: " << l.size());
        if (l.size() == 22)
        {
            DMACAsync async_msg;
            async_msg.header.stamp = ros::Time::now();
            async_msg.async = "usblphyp";
            KEYVALUE_HELPER(async_msg, "telegram_time", l);
            KEYVALUE_HELPER(async_msg, "measurement_time", l);
            KEYVALUE_HELPER(async_msg, "source_address", l);
            KEYVALUE_HELPER(async_msg, "type", l);
            KEYVALUE_HELPER(async_msg, "X123", l);
            KEYVALUE_HELPER(async_msg, "Y123", l);
            KEYVALUE_HELPER(async_msg, "Z123", l);
            KEYVALUE_HELPER(async_msg, "X432", l);
            KEYVALUE_HELPER(async_msg, "Y432", l);
            KEYVALUE_HELPER(async_msg, "Z432", l);
            KEYVALUE_HELPER(async_msg, "X341", l);
            KEYVALUE_HELPER(async_msg, "Y341", l);
            KEYVALUE_HELPER(async_msg, "Z341", l);
            KEYVALUE_HELPER(async_msg, "X412", l);
            KEYVALUE_HELPER(async_msg, "Y412", l);
            KEYVALUE_HELPER(async_msg, "Z412", l);
            KEYVALUE_HELPER(async_msg, "X153", l);
            KEYVALUE_HELPER(async_msg, "Y153", l);
            KEYVALUE_HELPER(async_msg, "Z153", l);
            KEYVALUE_HELPER(async_msg, "X254", l);
            KEYVALUE_HELPER(async_msg, "Y254", l);
            KEYVALUE_HELPER(async_msg, "Z254", l);
            publishAsync(async_msg);
        }
        else
        {
            ROS_ERROR_STREAM("" << __func__ << ": expected parameter count " << l.size() << " is not equal to 22.");
        }
    }
    
    void bitrate(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        static const boost::regex comma(",");
        std::list<std::string> l;
        boost::regex_split(std::back_inserter(l), parameters, comma);
        if (l.size() == 2)
        {
            DMACAsync async_msg;
            async_msg.header.stamp = ros::Time::now();
            async_msg.async = "bitrate";
            KEYVALUE_HELPER(async_msg, "direction", l);
            KEYVALUE_HELPER(async_msg, "value", l);
            publishAsync(async_msg);
        }
        else
        {
            ROS_ERROR_STREAM("" << __func__ << ": expected parameter count " << l.size() << " is not equal to 2.");
        }
    }

    void raddr(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        DMACAsync async_msg;
        async_msg.header.stamp = ros::Time::now();
        async_msg.async = "raddr";
        kv.key = "value";
        kv.value = parameters;
        async_msg.map.push_back(kv);
        publishAsync(async_msg);
    }
    
    void delivered(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        static const boost::regex comma(",");
        std::list<std::string> l;
        boost::regex_split(std::back_inserter(l), parameters, comma);
        if (l.size() == 2 || l.size() == 3)
        {
            DMACAsync async_msg;
            async_msg.header.stamp = ros::Time::now();
            std::string type = *(l.begin());
            l.pop_front();
            async_msg.async = "delivered" + boost::to_lower_copy<std::string>(type);
            if (type == "") {
                KEYVALUE_HELPER(async_msg, "counter", l);
            }
            KEYVALUE_HELPER(async_msg, "destination_address", l);
            publishAsync(async_msg);
        }
        else
        {
            ROS_ERROR_STREAM("" << __func__ << ": expected parameter count " << l.size() << " is not equal to 1 or 2.");
        }
    }
    
    void failed(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        static const boost::regex comma(",");
        std::list<std::string> l;
        boost::regex_split(std::back_inserter(l), parameters, comma);
        if (l.size() == 2 || l.size() == 3)
        {
            DMACAsync async_msg;
            async_msg.header.stamp = ros::Time::now();
            std::string type = *(l.begin());
            l.pop_front();
            async_msg.async = "failed" + boost::to_lower_copy<std::string>(type);
            if (type == "") {
                KEYVALUE_HELPER(async_msg, "counter", l);
            }
            KEYVALUE_HELPER(async_msg, "destination_address", l);
            publishAsync(async_msg);
        }
        else
        {
            ROS_ERROR_STREAM("" << __func__ << ": expected parameter count " << l.size() << " is not equal to 1 or 2.");
        }
    }
    
    void canceled(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        static const boost::regex comma(",");
        std::list<std::string> l;
        boost::regex_split(std::back_inserter(l), parameters, comma);
        if (l.size() == 2)
        {
            DMACAsync async_msg;
            async_msg.header.stamp = ros::Time::now();
            std::string type = *(l.begin());
            l.pop_front();
            async_msg.async = "canceled" + boost::to_lower_copy<std::string>(type);
            KEYVALUE_HELPER(async_msg, "destination_address", l);
            publishAsync(async_msg);
        }
        else
        {
            ROS_ERROR_STREAM("" << __func__ << ": expected parameter count " << l.size() << " is not equal to 1.");
        }
    }
    
    void expired(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        static const boost::regex comma(",");
        std::list<std::string> l;
        boost::regex_split(std::back_inserter(l), parameters, comma);
        if (l.size() == 2)
        {
            DMACAsync async_msg;
            async_msg.header.stamp = ros::Time::now();
            std::string type = *(l.begin());
            l.pop_front();
            async_msg.async = "expired" + boost::to_lower_copy<std::string>(type);
            KEYVALUE_HELPER(async_msg, "destination_address", l);
            publishAsync(async_msg);
        }
        else
        {
            ROS_ERROR_STREAM("" << __func__ << ": expected parameter count " << l.size() << " is not equal to 2.");
        }
    }
    
    void srclevel(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        DMACAsync async_msg;
        async_msg.header.stamp = ros::Time::now();
        async_msg.async = "srclevel";
        kv.key = "value";
        kv.value = parameters;
        async_msg.map.push_back(kv);
        publishAsync(async_msg);
    }

    void dropcnt(std::string parameters)
    {
        diagnostic_msgs::KeyValue kv;
        DMACAsync async_msg;
        async_msg.header.stamp = ros::Time::now();
        async_msg.async = "dropcnt";
        kv.key = "value";
        kv.value = parameters;
        async_msg.map.push_back(kv);
        publishAsync(async_msg);
    }
    
    void eclk(std::string parameters)
    {
        static const boost::regex clk_regex("^([^.]*)\\.([^,]*),([^,]*),([^,]*),([^,]*),([^.]*)\\.([^,]*)");
        boost::smatch clk_matches;

       if (boost::regex_match(parameters, clk_matches, clk_regex))
       { /* format matched */
           DMACClock clock_msg;
           clock_msg.stamp = ros::Time::now();
           clock_msg.mono.sec = boost::lexical_cast<unsigned int>(clk_matches[1]);
           clock_msg.mono.nsec = 1000 * boost::lexical_cast<unsigned int>(clk_matches[2]);
           clock_msg.phy_clock = boost::lexical_cast<unsigned int>(clk_matches[3]);
           clock_msg.phy_steer = boost::lexical_cast<unsigned int>(clk_matches[4]);
           clock_msg.status = boost::lexical_cast<unsigned int>(clk_matches[5]);
           clock_msg.utc_time.sec = boost::lexical_cast<unsigned int>(clk_matches[6]);
           clock_msg.utc_time.nsec = 1000 * boost::lexical_cast<unsigned int>(clk_matches[7]);
           publishClock(clock_msg);
       } 
       else
       {
           ROS_WARN_STREAM("ECLK parse error: " << parameters);
           more_.erase(0, more_.find("\r\n") + 2);
       }
    }
    
    void rcv_extract(std::string recv, int pid, int len, std::string tail)
    {
        if (recv == "RECV,") {
            recv_extract(pid, len, tail);
        } else if (recv == "RECVIM,") {
            recvim_extract(pid, len, tail);
        } else if (recv == "RECVIMS,") {
            recvims_extract(pid, len, tail);
        } else if (recv == "RECVPBM,") {
            recvpbm_extract(pid, len, tail);
        } else {
            ROS_ERROR_STREAM("Unsupported recv: " << recv);
            more_.erase(0, more_.find("\r\n") + 2);
        }
    }

    void recv_std_extract(std::string raw)
    { /* burst data publishing in std mode */

        if (!raw.empty()) {
            DMACPayload recv_msg;
            
            recv_msg.header.stamp = ros::Time::now();
            recv_msg.type = DMACPayload::DMAC_BURST;
            /* TODO: add remote address update tracking for std modem */
            recv_msg.payload = raw;
            publishRecv(recv_msg);
        }
    }
    
    void recv_extract(int pid, int len, std::string tail)
    {
       static const boost::regex recv_regex("^,([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),(.*)");
       boost::smatch recv_matches;

       DMACPayload recv_msg;

       if (boost::regex_match(tail, recv_matches, recv_regex))
       { /* format matched, check length */
           if (len + 2 <= recv_matches[8].str().length())
           {
               recv_msg.header.stamp = ros::Time::now();
               recv_msg.type = DMACPayload::DMAC_BURST;
               recv_msg.ack = false;
               recv_msg.force = false;
               recv_msg.source_address = boost::lexical_cast<int>(recv_matches[1]);
               recv_msg.destination_address = boost::lexical_cast<int>(recv_matches[2]);
               recv_msg.bitrate = boost::lexical_cast<int>(recv_matches[3]);
               recv_msg.rssi = boost::lexical_cast<int>(recv_matches[4]);
               recv_msg.integrity = boost::lexical_cast<int>(recv_matches[5]);
               recv_msg.propagation_time = boost::lexical_cast<double>(recv_matches[6]);
               recv_msg.relative_velocity = boost::lexical_cast<double>(recv_matches[7]);
               recv_msg.source_name = recv_matches[1];
               recv_msg.destination_name = recv_matches[2];

               std::string rest = recv_matches[8].str();
               boost::regex recv_payload_regex("^(.{" + boost::lexical_cast<std::string>(len) + "})\r\n(.*)");
               boost::smatch recv_payload_matches;
               if (boost::regex_match(rest, recv_payload_matches, recv_payload_regex))
               {
                   recv_msg.payload = recv_payload_matches[1].str();
                   more_ = recv_payload_matches[2].str();
                   publishRecv(recv_msg);
               }
               else
               {
                   ROS_ERROR_STREAM("Cannot extract payload of length: "
                                    << len << ": in "
                                    << recv_matches[8].str());
                   more_.erase(0, more_.find("\r\n") + 2);
               }
           }
           else
           {
               /* need more data */;
           }
       }
       else
       {
           ROS_WARN_STREAM("RECV parse error: " << tail);
           more_.erase(0, more_.find("\r\n") + 2);
       }
    }

    /* RECVIM[,p<pid>],len,src,dst,flag,dur,rssi,int,vel,data\r\n */
    /* match: publish recvim and put the rest to more_
     * more: put all to the more_
     * no match: generate error and empty more_
     */
    void recvim_extract(int pid, int len, std::string tail)
    {
       static const boost::regex recvim_regex("^,([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),(.*)");
       boost::smatch recvim_matches;

       DMACPayload recvim_msg;

       if (boost::regex_match(tail, recvim_matches, recvim_regex))
       { /* format matched, check length */
           if (len + 2 <= recvim_matches[8].str().length())
           {
               /* todo: add names parameter */
               recvim_msg.header.stamp = ros::Time::now()-ros::Duration(boost::lexical_cast<uint32_t>(recvim_matches[4])/1000000.0);
               recvim_msg.type = DMACPayload::DMAC_IM;
               recvim_msg.source_address = boost::lexical_cast<int>(recvim_matches[1]);
               recvim_msg.destination_address = boost::lexical_cast<int>(recvim_matches[2]);
               recvim_msg.duration = boost::lexical_cast<uint32_t>(recvim_matches[4]);
               recvim_msg.rssi = boost::lexical_cast<int>(recvim_matches[5]);
               recvim_msg.integrity = boost::lexical_cast<int>(recvim_matches[6]);
               recvim_msg.relative_velocity = boost::lexical_cast<double>(recvim_matches[7]);
               recvim_msg.source_name = recvim_matches[1];
               recvim_msg.destination_name = recvim_matches[2];
               /* todo: check lexical_cast with wrong data */
               if (recvim_matches[3] == "ack")
               {
                   recvim_msg.ack = true;
                   recvim_msg.force = false;
               }
               else if (recvim_matches[3] == "noack")
               {
                   recvim_msg.ack = false;
                   recvim_msg.force = false;
               }
               else if (recvim_matches[3] == "force")
               {
                   recvim_msg.ack = false;
                   recvim_msg.force = true;
               }
               else
               {
                   ROS_WARN_STREAM("Unsupported RECVIM ack flag: " << recvim_matches[3]);
                   more_.erase(0, more_.find("\r\n") + 2);
                   return;
               }
               std::string rest = recvim_matches[8].str();
               boost::regex recvim_payload_regex("^(.{" + boost::lexical_cast<std::string>(len) + "})\r\n(.*)");
               boost::smatch recvim_payload_matches;
               if (boost::regex_match(rest, recvim_payload_matches, recvim_payload_regex))
               {
                   recvim_msg.payload = recvim_payload_matches[1].str();
                   more_ = recvim_payload_matches[2].str();
                   publishRecv(recvim_msg);
               }
               else
               {
                   ROS_ERROR_STREAM("Cannot extract payload of length: "
                                    << len << ": in "
                                    << recvim_matches[8].str());
                   more_.erase(0, more_.find("\r\n") + 2);
               }
           }
           else
           {
               /* need more data */;
           }
       }
       else
       {
           ROS_WARN_STREAM("RECVIM parse error: " << tail);
           more_.erase(0, more_.find("\r\n") + 2);
       }
    }

    void recvims_extract(int pid, int len, std::string tail)
    {
        static const boost::regex recvims_regex("^,([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),(.*)");
        boost::smatch recvims_matches;
        
        DMACPayload recvims_msg;
        
        if (boost::regex_match(tail, recvims_matches, recvims_regex))
        { /* format matched, check length */
            if (len + 2 <= recvims_matches[8].str().length())
            {
                recvims_msg.header.stamp = ros::Time::now()-ros::Duration(boost::lexical_cast<uint32_t>(recvims_matches[4])/1000000.0);
                recvims_msg.type = DMACPayload::DMAC_IMS;
                recvims_msg.source_address = boost::lexical_cast<int>(recvims_matches[1]);
                recvims_msg.destination_address = boost::lexical_cast<int>(recvims_matches[2]);
                recvims_msg.timestamp = boost::lexical_cast<uint32_t>(recvims_matches[3]);
                recvims_msg.duration = boost::lexical_cast<uint32_t>(recvims_matches[4]);
                recvims_msg.rssi = boost::lexical_cast<int>(recvims_matches[5]);
                recvims_msg.integrity = boost::lexical_cast<int>(recvims_matches[6]);
                recvims_msg.relative_velocity = boost::lexical_cast<double>(recvims_matches[7]);
                recvims_msg.source_name = recvims_matches[1];
                recvims_msg.destination_name = recvims_matches[2];
                std::string rest = recvims_matches[8].str();
                boost::regex recv_payload_regex("^(.{" + boost::lexical_cast<std::string>(len) + "})\r\n(.*)");
                boost::smatch recv_payload_matches;
                if (boost::regex_match(rest, recv_payload_matches, recv_payload_regex))
                {
                    recvims_msg.payload = recv_payload_matches[1].str();
                    more_ = recv_payload_matches[2].str();
                    publishRecv(recvims_msg);
                }
                else
                {
                    ROS_ERROR_STREAM("Cannot extract payload of length: "
                                     << len << ": in "
                                     << recvims_matches[8].str());
                    more_.erase(0, more_.find("\r\n") + 2);
                }
            }
            else
            {
                /* need more data */;
            }
        }
        else
        {
            ROS_WARN_STREAM("RECVIMS parse error: " << tail);
            more_.erase(0, more_.find("\r\n") + 2);
        }
    }

    void recvpbm_extract(int pid, int len, std::string tail)
    {
        static const boost::regex recvpbm_regex("^,([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),(.*)");
        boost::smatch recvpbm_matches;
        
        DMACPayload recvpbm_msg;
        
        if (boost::regex_match(tail, recvpbm_matches, recvpbm_regex))
        { /* format matched, check length */
            if (len + 2 <= recvpbm_matches[7].str().length())
            {
                recvpbm_msg.header.stamp = ros::Time::now()-ros::Duration(boost::lexical_cast<uint32_t>(recvpbm_matches[3])/1000000.0);
                recvpbm_msg.type = DMACPayload::DMAC_PBM;
                recvpbm_msg.source_address = boost::lexical_cast<int>(recvpbm_matches[1]);
                recvpbm_msg.destination_address = boost::lexical_cast<int>(recvpbm_matches[2]);
                recvpbm_msg.duration = boost::lexical_cast<uint32_t>(recvpbm_matches[3]);
                recvpbm_msg.rssi = boost::lexical_cast<int>(recvpbm_matches[4]);
                recvpbm_msg.integrity = boost::lexical_cast<int>(recvpbm_matches[5]);
                recvpbm_msg.relative_velocity = boost::lexical_cast<double>(recvpbm_matches[6]);
                recvpbm_msg.source_name = recvpbm_matches[1];
                recvpbm_msg.destination_name = recvpbm_matches[2];
                std::string rest = recvpbm_matches[7].str();
                boost::regex recv_payload_regex("^(.{" + boost::lexical_cast<std::string>(len) + "})\r\n(.*)");
                boost::smatch recv_payload_matches;
                if (boost::regex_match(rest, recv_payload_matches, recv_payload_regex))
                {
                    recvpbm_msg.payload = recv_payload_matches[1].str();
                    more_ = recv_payload_matches[2].str();
                    publishRecv(recvpbm_msg);
                }
                else
                {
                    ROS_ERROR_STREAM("Cannot extract payload of length: "
                                     << len << ": in "
                                     << recvpbm_matches[7].str());
                    more_.erase(0, more_.find("\r\n") + 2);
                }
            }
            else
            {
                /* need more data */;
            }
        }
        else
        {
            ROS_WARN_STREAM("RECVPBM parse error: " << tail);
            more_.erase(0, more_.find("\r\n") + 2);
        }
    }
    
};

}  // namespace

#endif  // DMAC_PARSER_H
