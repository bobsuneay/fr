// Transport regression with a fake Modbus device; no ROS or real hardware required.
#include "hkv_gripper_controller/default_driver.hpp"
#include "hkv_gripper_controller/modbus_utils.hpp"
#include <cassert>
#include <map>
#include <stdexcept>
using namespace hkv_gripper_controller;

struct Bus : Serial {
  bool opened=false, fail=false, wrong_slave=false;
  std::vector<uint8_t> pending;
  std::map<uint16_t,uint16_t> regs;
  std::vector<uint16_t> writes;
  bool open() override { opened=true; return true; }
  void close() override { opened=false; }
  bool isOpen() const override { return opened; }
  void flush() override { pending.clear(); }
  size_t available() const override { return pending.size(); }
  std::vector<uint8_t> read(size_t) override { auto p=pending; pending.clear(); return p; }
  size_t write(const std::vector<uint8_t>& f) override {
    if(fail) return 0;
    auto addr=static_cast<uint16_t>((f[2]<<8)|f[3]);
    auto n=static_cast<uint16_t>((f[4]<<8)|f[5]);
    if(f[1]==6) {
      writes.push_back(addr); regs[addr]=n;
      pending.assign(f.begin(),f.begin()+6);
    } else if(f[1]==16) {
      writes.push_back(addr);
      for(unsigned i=0;i<n;i++) regs[addr+i]=(f[7+2*i]<<8)|f[8+2*i];
      pending.assign(f.begin(),f.begin()+6);
    } else {
      pending={f[0],3,static_cast<uint8_t>(2*n)};
      for(unsigned i=0;i<n;i++) { auto v=regs[addr+i]; pending.push_back(v>>8); pending.push_back(v&255); }
    }
    if(wrong_slave) pending[0]=2;
    auto crc=modbus_utils::calculateCRC16(pending);
    pending.push_back(crc&255); pending.push_back(crc>>8);
    return f.size();
  }
};

int main() {
  auto bus=std::make_shared<Bus>();
  DefaultDriver driver(bus);
  assert(driver.connect());
  assert(bus->writes.empty());
  driver.grip(uint8_t(0),uint8_t(0),uint8_t(0));
  assert(bus->regs[0x12]==1); // actual force register is programmed
  assert(bus->regs[0x0f]==200 && bus->regs[0x10]==100);
  driver.grip(uint8_t(255),uint8_t(255),uint8_t(0));
  assert(bus->regs[0x10]==0 && bus->regs[0x0f]==1500);
  bus->regs[8]=50; bus->regs[0]=20; bus->regs[3]=30;
  auto state=driver.readFingerState();
  assert(state.is_valid && state.position_register==50 && state.x1_value==20 && state.x2_value==30);
  bus->wrong_slave=true;
  assert(!driver.readFingerState().is_valid);
  bus->wrong_slave=false; bus->fail=true;
  bool threw=false;
  try { driver.grip(uint8_t(0),uint8_t(0),uint8_t(0)); } catch(const std::runtime_error&) { threw=true; }
  assert(threw);
  const auto count=bus->writes.size();
  driver.disconnect();
  assert(bus->writes.size()==count); // disconnect sends no jaw-open command
}
