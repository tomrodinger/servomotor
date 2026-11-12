// Exercises the real generated methods AND real Communication.cpp entirely offline.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>
#include <initializer_list>
#include "Servomotor.h"
ConsoleSerial Serial;
HardwareSerial Serial1;
uint32_t TestClock::now = 0;
static unsigned checks, failures;
static constexpr uint64_t UID = UINT64_C(0x0123456789ABCDEF);
static void check(bool ok, const char* what) {
    ++checks; if(!ok) { ++failures; std::fprintf(stderr,"FAIL %s\n",what); }
}
static void le(std::vector<uint8_t>& bytes,uint64_t value,unsigned n) {
    for(unsigned i=0;i<n;++i)bytes.push_back(static_cast<uint8_t>(value>>(8*i)));
}
static uint32_t crc(const std::vector<uint8_t>& bytes) {
    uint32_t value=UINT32_MAX;
    for(uint8_t byte:bytes) { value^=byte; for(unsigned i=0;i<8;++i)value=(value>>1)^((value&1)?0xedb88320u:0); }
    return ~value;
}
static std::vector<uint8_t> reply(std::vector<uint8_t> payload={},bool crcOn=true,int error=0,bool ack=false,bool ext=false) {
    unsigned size=(ext?3:1)+1+(ack?0:1)+payload.size()+(crcOn?4:0);
    std::vector<uint8_t> p;
    if(ext) { p.push_back(255);le(p,size,2); } else p.push_back((size<<1)|1);
    p.push_back(crcOn?253:252);if(!ack)p.push_back(error);
    p.insert(p.end(),payload.begin(),payload.end());if(crcOn)le(p,crc(p),4);return p;
}
static std::vector<uint8_t> request(uint64_t address,int command,std::vector<uint8_t> payload,bool crcOn) {
    std::vector<uint8_t> body;
    if(address>255) { body.push_back(254);le(body,address,8); } else body.push_back(address);
    body.push_back(command);body.insert(body.end(),payload.begin(),payload.end());
    std::vector<uint8_t> p{static_cast<uint8_t>(((body.size()+1+(crcOn?4:0))<<1)|1)};
    p.insert(p.end(),body.begin(),body.end());if(crcOn)le(p,crc(p),4);return p;
}
static std::vector<uint8_t> position(int64_t p) { std::vector<uint8_t> b;le(b,static_cast<uint64_t>(p),8);return b; }
static unsigned framesSent() {
    unsigned frames=0;size_t i=0;
    while(i<Serial1.tx.size()) { unsigned size=Serial1.tx[i]>>1;if(size==127)return 999;if(!size||i+size>Serial1.tx.size())return 999;i+=size;++frames; }
    return frames;
}
static void prepare(Servomotor& motor,bool crcOn,uint64_t address,const std::vector<uint8_t>& response,unsigned fragmentMs=0) {
    Serial1.reset();if(crcOn)motor.enableCRC32();else motor.disableCRC32();
    if(address>255)motor.useUniqueId(address);else motor.useAlias(address);
    if(fragmentMs) for(size_t i=0;i<response.size();++i)Serial1.schedule(i*fragmentMs,{response[i]});
    else Serial1.schedule(0,response);
}
static void basic(Servomotor& m) {
    const int64_t values[]={INT64_MIN,INT64_MIN+1,-(INT64_C(1)<<53)-1,-123,-10,-1,0,10,123,(INT64_C(1)<<53)+1,INT64_MAX};
    for(bool crcOn:{false,true})for(uint64_t address:{UINT64_C(88),UID})for(int64_t target:values) {
        int64_t measured=target==INT64_MIN?INT64_MAX:target-1;
        prepare(m,crcOn,address,reply(position(measured),crcOn));
        int64_t actual=m.setImmediatePositionTargetAndGetPositionRaw(target);
        check(m.getError()==0&&actual==measured,"raw configured-address measured position");
        check(Serial1.tx==request(address,50,position(target),crcOn),"exact command 50 wire frame");
        check(framesSent()==1,"one frame per combined call");
        prepare(m,crcOn,address,reply(position(measured),crcOn),2);
        actual=m.setImmediatePositionTargetAndGetPositionRaw(UID,target);
        check(m.getError()==0&&actual==measured,"raw explicit UID fragmented reply");
        check(Serial1.tx==request(UID,50,position(target),crcOn),"explicit UID overrides configured alias");
        prepare(m,crcOn,address,reply({},crcOn,0,true));m.setImmediatePositionTargetRaw(target);
        check(m.getError()==0&&Serial1.tx==request(address,49,position(target),crcOn),"compact ACK command 49");
        prepare(m,crcOn,address,reply({},crcOn,0,true));m.setImmediatePositionTargetRaw(UID,target);
        check(m.getError()==0&&Serial1.tx==request(UID,49,position(target),crcOn),"explicit UID command 49");
    }
}
static void pid(Servomotor& m) {
    const uint32_t triplets[][3]={{0,0,0},{2000,5,175000},{0x7fffffff,0x80000000,0xffffffff},{0x01234567,0x89abcdef,0xfedcba98}};
    for(bool crcOn:{false,true})for(uint64_t address:{UINT64_C(88),UID})for(const auto& t:triplets)for(bool explicitUid:{false,true}) {
        std::vector<uint8_t> data;for(auto v:t)le(data,v,4);
        prepare(m,crcOn,address,reply(data,crcOn),1);
        auto p=explicitUid?m.getPidConstants(UID):m.getPidConstants();
        check(m.getError()==0&&p.kP==t[0]&&p.kI==t[1]&&p.kD==t[2],"PID preserves full raw uint32 triplet");
        check(Serial1.tx==request(explicitUid?UID:address,48,{},crcOn),"PID zero-payload request");
    }
}
static void failuresAndRecovery(Servomotor& m) {
    for(bool crcOn:{false,true})for(int command:{48,49,50})for(int error:{8,13,19,27,45,51,55}) {
        prepare(m,crcOn,UID,reply({},crcOn,error));
        if(command==48)m.getPidConstants();else if(command==49)m.setImmediatePositionTargetRaw(123);else m.setImmediatePositionTargetAndGetPositionRaw(123);
        check(m.getError()==error,"fatal error returned exactly");check(framesSent()==1,"fatal reply is not retried");
    }
    for(bool crcOn:{false,true})for(unsigned size:{0u,1u,7u,9u,12u,16u,24u}) {
        prepare(m,crcOn,88,reply(std::vector<uint8_t>(size,0),crcOn));
        m.setImmediatePositionTargetAndGetPositionRaw(1);
        check(m.getError()!=0,"bad/legacy response size rejected");check(framesSent()==1,"bad response never resends motion");
    }
    for(int command:{48,49,50}) {
        auto frame=reply(command==48?std::vector<uint8_t>(12,0):command==50?position(10):std::vector<uint8_t>{},true,0,command==49);
        auto corrupt=frame;corrupt.back()^=0x80;prepare(m,true,88,corrupt);
        if(command==48)m.getPidConstants();else if(command==49)m.setImmediatePositionTargetRaw(1);else m.setImmediatePositionTargetAndGetPositionRaw(1);
        check(m.getError()==COMMUNICATION_ERROR_CRC32_MISMATCH,"corrupt CRC rejected");check(framesSent()==1,"bad CRC not retried");
        for(size_t cut=0;cut<frame.size();++cut) {
            prepare(m,true,88,std::vector<uint8_t>(frame.begin(),frame.begin()+cut));
            if(command==48)m.getPidConstants();else if(command==49)m.setImmediatePositionTargetRaw(1);else m.setImmediatePositionTargetAndGetPositionRaw(1);
            check(m.getError()!=0,"all truncated frames fail");check(TestClock::now<=1100,"truncation has bounded timeout");check(framesSent()==1,"truncation does not retry");
        }
    }
    // A device error byte is not trustworthy until its CRC has been checked.
    for (int command : {48, 49, 50}) {
        auto damaged = reply({}, true, 55); damaged.back() ^= 1;
        prepare(m, true, UID, damaged);
        if(command==48)m.getPidConstants();else if(command==49)m.setImmediatePositionTargetRaw(0);else m.setImmediatePositionTargetAndGetPositionRaw(0);
        check(m.getError()==COMMUNICATION_ERROR_CRC32_MISMATCH, "corrupt fatal reply is a CRC error");
        auto truncated = reply({}, true, 55); truncated.pop_back();
        prepare(m, true, UID, truncated);
        if(command==48)m.getPidConstants();else if(command==49)m.setImmediatePositionTargetRaw(0);else m.setImmediatePositionTargetAndGetPositionRaw(0);
        check(m.getError()==COMMUNICATION_ERROR_TIMEOUT, "incomplete fatal reply is a timeout");
    }
    // A late prefix must not give the missing CRC an entirely new timeout.
    auto p=reply(position(123));p.resize(p.size()-4);prepare(m,true,88,{});Serial1.schedule(900,p);
    m.setImmediatePositionTargetAndGetPositionRaw(1);
    check(m.getError()==COMMUNICATION_ERROR_TIMEOUT,"late missing CRC times out");
    check(TestClock::now<=1050,"whole-packet deadline includes CRC stage");
    prepare(m,true,88,reply(position(456)));check(m.setImmediatePositionTargetAndGetPositionRaw(1)==456&&m.getError()==0,"next valid response clears previous error");
    for(bool crcOn:{false,true}) {
        prepare(m,crcOn,255,{});m.setImmediatePositionTargetRaw(123);
        check(framesSent()==1&&m.getError()==COMMUNICATION_ERROR_TIMEOUT,"broadcast is silent and never retried");
    }
}
static void corruptedFrames(Servomotor& m) {
    for (int command : {48,49,50}) for (bool fault : {false,true}) {
        auto valid = fault ? reply({},true,55) : reply(command==48?std::vector<uint8_t>(12,0):command==50?position(10):std::vector<uint8_t>{},true,0,command==49);
        for (size_t byte=0; byte<valid.size(); ++byte) for (unsigned bit=0; bit<8; ++bit) {
            auto damaged=valid; damaged[byte]^=1u<<bit;
            prepare(m,true,UID,damaged);
            if(command==48)m.getPidConstants();else if(command==49)m.setImmediatePositionTargetRaw(0);else m.setImmediatePositionTargetAndGetPositionRaw(0);
            check(m.getError()<0,"every single-bit corruption is a transport error");
            check(framesSent()==1,"corrupted packet never retries motion");
        }
    }
}

static void randomAndRecovery(Servomotor& m) {
    uint64_t seed = UINT64_C(0x495017130);
    for (unsigned i=0; i<1000; ++i) {
        seed ^= seed << 13; seed ^= seed >> 7; seed ^= seed << 17;
        int64_t target; std::memcpy(&target, &seed, sizeof(target));
        int64_t measured = ~target;
        bool crcOn = i % 2; uint64_t address = i % 3 ? UID : 88;
        prepare(m,crcOn,address,reply(position(measured),crcOn,0,false,i%4==0));
        auto actual=m.setImmediatePositionTargetAndGetPositionRaw(target);
        check(actual==measured&&m.getError()==0,"random full-width measured position and extended header");
        check(Serial1.tx==request(address,50,position(target),crcOn),"random full-width exact request");
    }
    for (bool crcOn : {false,true}) {
        auto first=reply(position(10),crcOn),second=reply(position(-10),crcOn);
        first.insert(first.end(),second.begin(),second.end());prepare(m,crcOn,UID,first);
        check(m.setImmediatePositionTargetAndGetPositionRaw(100)==10&&m.getError()==0,"coalesced first reply");
        check(m.setImmediatePositionTargetAndGetPositionRaw(-100)==-10&&m.getError()==0,"coalesced next reply preserved");
        first=reply(std::vector<uint8_t>(24,0),crcOn);first.insert(first.end(),second.begin(),second.end());
        prepare(m,crcOn,UID,first);m.setImmediatePositionTargetAndGetPositionRaw(0);
        check(m.getError()!=0,"legacy payload rejected before recovery");
        check(m.setImmediatePositionTargetAndGetPositionRaw(0)==-10&&m.getError()==0,"wrong size frame drained without swallowing next reply");
    }
    prepare(m,true,UID,{});TestClock::now=UINT32_MAX-500;
    m.setImmediatePositionTargetAndGetPositionRaw(0);
    check(m.getError()==COMMUNICATION_ERROR_TIMEOUT&&TestClock::now<600,"timeout works across millis wrap");
}

static void conversions(Servomotor& m) {
    for(bool explicitUid:{false,true}) {
        m.setPositionUnit(PositionUnit::DEGREES);prepare(m,true,88,reply(position(-819200)));
        float value=explicitUid?m.setImmediatePositionTargetAndGetPosition(UID,90):m.setImmediatePositionTargetAndGetPosition(90);
        check(m.getError()==0&&std::fabs(value+90)<0.0001f,"converted reply uses measured position");
        check(Serial1.tx==request(explicitUid?UID:88,50,position(819200),true),"converted target encoded correctly");
        prepare(m,true,88,reply({},true,0,true));
        if(explicitUid)m.setImmediatePositionTarget(UID,-90);else m.setImmediatePositionTarget(-90);
        check(m.getError()==0,"converted standalone target accepted");
        check(Serial1.tx==request(explicitUid?UID:88,49,position(-819200),true),"converted standalone wire units and UID");
        m.setPositionUnit(PositionUnit::ENCODER_COUNTS);
        for (float good : {-std::ldexp(1.0f,63), std::nextafter(std::ldexp(1.0f,63),0.0f)}) {
            prepare(m,true,88,reply(position(0)));
            if(explicitUid)m.setImmediatePositionTargetAndGetPosition(UID,good);else m.setImmediatePositionTargetAndGetPosition(good);
            check(m.getError()==0&&Serial1.tx==request(explicitUid?UID:88,50,position(static_cast<int64_t>(good)),true),"representable converted i64 boundary accepted exactly");
        }
        for(float bad:{std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN(),std::ldexp(1.0f,63),std::nextafter(-std::ldexp(1.0f,63),-std::numeric_limits<float>::infinity()),-std::ldexp(1.0f,64)}) {
            prepare(m,true,88,reply(position(0)));
            if(explicitUid)m.setImmediatePositionTargetAndGetPosition(UID,bad);else m.setImmediatePositionTargetAndGetPosition(bad);
            check(m.getError()<0&&Serial1.tx.empty(),"invalid converted combined target never transmitted");
            prepare(m,true,88,reply({},true,0,true));
            if(explicitUid)m.setImmediatePositionTarget(UID,bad);else m.setImmediatePositionTarget(bad);
            check(m.getError()<0&&Serial1.tx.empty(),"invalid converted standalone target never transmitted");
        }
    }
}
int main() {
    Servomotor m(88,Serial1);basic(m);pid(m);failuresAndRecovery(m);corruptedFrames(m);randomAndRecovery(m);conversions(m);
    std::printf("%s: %u real-transport command 48/49/50 checks; %u failures\n",failures?"FAILED":"PASSED",checks,failures);
    return failures?1:0;
}
