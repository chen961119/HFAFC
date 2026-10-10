#include "parameter_record.h"
#include "parameter_eeprom.h"
#include <EEPROM.h>
#include <string.h>

namespace {
constexpr size_t SIZE=2048, HEADER=24;
constexpr int BASE=128;
constexpr uint16_t VERSION=3;
constexpr uint32_t MAGIC=0x43504531u, COMMIT=0x434f464cu;
uint8_t current[SIZE], next[SIZE];
int active=-1;
const char *state="DEFAULTS";
uint16_t get16(const uint8_t *p) { return uint16_t(p[0]) | uint16_t(p[1])<<8; }
uint32_t get32(const uint8_t *p) { return uint32_t(get16(p)) | uint32_t(get16(p+2))<<16; }
void put16(uint8_t *p,uint16_t v) { p[0]=v;p[1]=v>>8; }
void put32(uint8_t *p,uint32_t v) { put16(p,v);put16(p+2,v>>16); }
uint32_t crc(const uint8_t *data,size_t used) {
  uint32_t value=UINT32_MAX;
  for (size_t i=0;i<used;++i) {
    if (i>=16 && i<HEADER) continue;
    value^=data[i];
    for (unsigned bit=0;bit<8;++bit)
      value=value>>1 ^ (0xedb88320u & (0u-(value&1u)));
  }
  return ~value;
}
bool readLine(const uint8_t *data,size_t used,size_t &offset,char *line) {
  size_t length=0;
  while (offset<used && data[offset]!='\n') {
    if (!data[offset] || length+1>=PARAMETER_RECORD_SIZE) return false;
    line[length++]=data[offset++];
  }
  if (offset==used) return false;
  ++offset;line[length]=0;return true;
}
bool valid(const uint8_t *data) {
  const uint16_t version=get16(data+4);
  const size_t rows=get16(data+6),used=get16(data+12);
  if (get32(data)!=MAGIC || version!=VERSION || get32(data+20)!=COMMIT ||
      used<HEADER || used>SIZE || get16(data+14)!=0 || get32(data+16)!=crc(data,used)) return false;
  size_t offset=HEADER;
  for (size_t i=0;i<rows;++i) {
    char line[PARAMETER_RECORD_SIZE],*name,*type,*value;
    if (!readLine(data,used,offset,line) || !ParameterRecord::split(line,name,type,value)) return false;
    float f=0;int32_t integer=0;double decoded;
    FlightParameter p(0,name,&f,-3.4028234663852886e38,3.4028234663852886e38,"","");
    if (!strcmp(type,"int")) { p.integer=&integer;p.minimum=INT32_MIN;p.maximum=INT32_MAX; }
    if (!ParameterRecord::parseValue(p,value,decoded)) return false;
    size_t previous=HEADER;
    for (size_t j=0;j<i;++j) {
      char other[PARAMETER_RECORD_SIZE],*n,*t,*v;
      if (!readLine(data,used,previous,other) || !ParameterRecord::split(other,n,t,v) || !strcmp(n,name)) return false;
    }
  }
  return offset==used;
}
void readSlot(int slot,uint8_t *buffer) {
  for (size_t i=0;i<SIZE;++i) buffer[i]=EEPROM.read(BASE+slot*SIZE+i);
}

}

bool initializeEEPROMParameters() {
  active=-1;state="DEFAULTS";
  if (EEPROM.length()<BASE+2*SIZE) { state="CAPACITY_ERROR";return false; }
  readSlot(0,current);readSlot(1,next);
  const bool a=valid(current),b=valid(next);
  if (!a && !b) {
    if (get32(current)==MAGIC || get32(next)==MAGIC) state="CORRUPT";
    return false;
  }
  const uint32_t difference=get32(next+8)-get32(current+8);
  if (b && (!a || (difference && difference<0x80000000u))) { memcpy(current,next,SIZE);active=1; }
  else active=0;
  state="LOADED";
  return true;
}

bool readEEPROMParameter(const FlightParameter &p,double &value) {
  if (active<0 || !validParameterName(p.name)) return false;
  size_t offset=HEADER;
  for (size_t i=0;i<get16(current+6);++i) {
    char line[PARAMETER_RECORD_SIZE],*name,*type,*text;
    if (!readLine(current,get16(current+12),offset,line) || !ParameterRecord::split(line,name,type,text)) return false;
    if (!strcmp(name,p.name) && !strcmp(type,p.type())) return ParameterRecord::parseValue(p,text,value);
  }
  return false;
}

bool hasEEPROMMigrationSource(const FlightParameter &p) {
  double value;return readEEPROMParameter(p,value);
}

bool saveEEPROMParameters(const FlightParameter *table,size_t count,const double *values,bool sdVerified) {
  if (EEPROM.length()<BASE+2*SIZE) return false;
  memset(next,0,SIZE);
  size_t rows=0,used=HEADER;
  // Rebuild from the current registry: same name:type=value text as SD, without IDs or status.
  // Keep an SD migration source only until the target file has been verified.
  for (size_t i=0;i<count;++i) {
    const auto &p=table[i];
    double value=values[i];
    if (p.storage!=ParameterStorage::EEPROM) {
      if (sdVerified || !hasEEPROMMigrationSource(p) || !readEEPROMParameter(p,value)) continue;
    }
    for (size_t j=0;j<i;++j) if (!strcmp(table[j].name,p.name)) return false;
    char line[PARAMETER_RECORD_SIZE];
    const int n=ParameterRecord::encode(p,value,line,sizeof(line));
    if (n<0 || static_cast<size_t>(n)>SIZE-used) return false;
    memcpy(next+used,line,n);used+=n;++rows;
  }
  put32(next,MAGIC);put16(next+4,VERSION);put16(next+6,rows);
  put32(next+8,active>=0 ? get32(current+8)+1 : 1);
  put16(next+12,used);put16(next+14,0);
  put32(next+16,crc(next,used));put32(next+20,COMMIT);
  if (active>=0 && get16(current+4)==VERSION && rows==get16(current+6) && used==get16(current+12) &&
      !memcmp(next+HEADER,current+HEADER,used-HEADER)) return true;
  const int slot=active==0 ? 1 : 0,base=BASE+slot*SIZE;
  for (unsigned i=0;i<4;++i) EEPROM.update(base+20+i,0);
  for (size_t i=0;i<used;++i) {
    if (i>=20 && i<HEADER) continue;
    EEPROM.update(base+i,next[i]);
    if (EEPROM.read(base+i)!=next[i]) return false;
  }
  for (unsigned i=0;i<4;++i) EEPROM.update(base+20+i,next[20+i]);
  for (size_t i=0;i<used;++i) if (EEPROM.read(base+i)!=next[i]) return false;
  if (!valid(next)) return false;
  memcpy(current,next,SIZE);active=slot;state="SAVED";return true;
}

const char *eepromParameterState() { return state; }
