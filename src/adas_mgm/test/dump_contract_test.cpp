#include <sstream>
#include <iostream>
#include "tools/dump_reader.hpp"
using namespace adas_mgm;
int main() {
  DumpHeader h{kDumpMagic,kDumpVersion,sizeof(CoreSnapshot),sizeof(CoreParams),{}};
  CoreSnapshot snapshot{};
  std::ostringstream output(std::ios::binary);
  output.write(reinterpret_cast<const char *>(&h),sizeof(h));
  output.write(reinterpret_cast<const char *>(&snapshot),sizeof(snapshot));
  const auto good=output.str();
  int failures=0;
  auto check=[&](std::string bytes,bool expected,const char *name) {
    std::istringstream input(bytes,std::ios::binary); DumpHeader parsed{};
    if(read_dump_header(input,parsed)!=expected) {std::cerr<<name<<'\n'; ++failures;}
  };
  check(good,true,"v35 round trip");
  auto field=[&](unsigned index,uint32_t value) {
    auto bytes=good; bytes.replace(index*4,4,reinterpret_cast<const char *>(&value),4); return bytes;
  };
  check(field(0,0),false,"bad magic");
  check(field(1,32),false,"ambiguous v32");
  check(field(1,33),false,"old v33 semantics");
  check(field(1,34),false,"old v34 optional prepare semantics");
  check(field(1,kDumpVersion+1),false,"future version");
  check(field(2,sizeof(CoreSnapshot)-1),false,"snapshot ABI mismatch");
  check(field(3,sizeof(CoreParams)-4),false,"missing params cannot be zero filled");
  check(good.substr(0,12),false,"partial header");
  check(good.substr(0,sizeof(h)-1),false,"partial params");
  check(good.substr(0,good.size()-1),false,"partial snapshot");
  check(good+"x",false,"trailing byte");
  return failures?1:0;
}
