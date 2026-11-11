// steem_dump.cpp - list every ID/data field that Steem SSE with HFEv3's own
// HFE decoder (TImageHFE, disk_hfe.cpp) yields, in the same format as
// "hfe_model -s", so the two decoders can be diffed. GPLv3.
// Built against Steem's tests/hfe mocks: see the Makefile one level up.
#include <algorithm>
#include <string>
#include "pch.h"
#include "disk_hfe.h"
#include <vector>

int g_trace=0;
EasyStr RunDir(".");
TFloppyDisk FloppyDisk[3];
TFloppyDrive FloppyDrive[3];
TFdcMock Fdc;
TImageHFE ImageHFE[3];
long GetFileLength(FILE*f){long p=ftell(f);fseek(f,0,SEEK_END);long l=ftell(f);fseek(f,p,SEEK_SET);return l;}
WORD TImageMfm::GetMfmData(WORD p){if(p!=0xFFFF)Position=p%FloppyDisk[Id].TrackBytes;return GetImageWord(Position);}
void TImageMfm::IncPosition(){Position=(Position+1)%FloppyDisk[Id].TrackBytes;}
void TImageMfm::SetMfmData(WORD,WORD){FloppyDisk[Id].WrittenTo=true;}

// ---- cross-check dump: every ID/data field Steem's HFE decoder yields,
// in the same format as hfe_model -s
static WORD crc16(WORD c,BYTE b){c^=(WORD)b<<8;for(int i=0;i<8;i++)c=(c&0x8000)?(WORD)(c<<1^0x1021):(WORD)(c<<1);return c;}
static unsigned fnv(const BYTE*p,int n){unsigned h=2166136261u;for(int i=0;i<n;i++){h^=p[i];h*=16777619u;}return h;}
static int byte_at(const std::vector<BYTE>&b,size_t end){int v=0;for(int k=0;k<8;k++)v=v<<1|b[(end-14+2*k)%b.size()];return v;}

int main(int argc,char**argv){
  for(int a=1;a<argc;a++){
    TImageHFE&im=ImageHFE[0];
    if(!im.Open(argv[a])){printf("%s: open failed\n",argv[a]);continue;}
    int nt=im.file_header->number_of_track, ns=im.file_header->number_of_side;
    for(int t=0;t<nt;t++)for(int s=0;s<ns;s++){
      if(!im.LoadTrack(s,t)) continue;
      int nw=FloppyDisk[0].TrackBytes;
      std::vector<BYTE> bits;
      for(int w=0;w<nw;w++){WORD m=im.GetImageWord(w);for(int k=15;k>=0;k--)bits.push_back((m>>k)&1);}
      size_t n=bits.size(); if(n<64) continue;
      // marks over two turns, keep those starting in the first turn
      std::vector<size_t> a1; unsigned sh=0; size_t last=0; bool have=false;
      for(size_t i=0;i<2*n;i++){sh=sh<<1|bits[i%n];
        if((sh&0xffff)==0x4489 && (!have||i-last>=16)){a1.push_back(i);last=i;have=true;}}
      std::vector<std::string> lines;
      for(size_t i=0;i+2<a1.size();i++){
        if(a1[i+1]-a1[i]!=16||a1[i+2]-a1[i+1]!=16) continue;
        size_t p=a1[i+2]+16; if(p>=n+16*0 && a1[i]>=n) continue;
        if(a1[i]>=n) continue;
        if(byte_at(bits,p)!=0xFE) continue;
        int b[6]; for(int k=0;k<6;k++) b[k]=byte_at(bits,p+16*(k+1));
        WORD c=0xffff; c=crc16(c,0xA1);c=crc16(c,0xA1);c=crc16(c,0xA1);c=crc16(c,0xFE);
        for(int k=0;k<4;k++) c=crc16(c,(BYTE)b[k]);
        int idok=c==(WORD)(b[4]<<8|b[5]);
        int dam=-1,datok=0; unsigned h=0; size_t id_end=p+16*6;
        for(size_t j=i+3;j+2<a1.size();j++){
          if(a1[j]<=id_end) continue;
          if(a1[j]>id_end+16*(43+3)) break;
          if(a1[j+1]-a1[j]!=16||a1[j+2]-a1[j+1]!=16) continue;
          size_t q=a1[j+2]+16; int d=byte_at(bits,q);
          if(d<0xF8||d>0xFB){ if(d==0xFE) break; continue; }
          dam=d; int size=128<<(b[3]&3); std::vector<BYTE> data;
          WORD c2=0xffff; c2=crc16(c2,0xA1);c2=crc16(c2,0xA1);c2=crc16(c2,0xA1);c2=crc16(c2,(BYTE)d);
          for(int k=0;k<size;k++){int v=byte_at(bits,q+16*(k+1));data.push_back((BYTE)v);c2=crc16(c2,(BYTE)v);}
          int hi=byte_at(bits,q+16*(size+1)),lo=byte_at(bits,q+16*(size+2));
          datok=c2==(WORD)(hi<<8|lo); h=fnv(&data[0],size); break;
        }
        char ln[160]; snprintf(ln,sizeof ln,"T%02d.%d id %d.%d.%d.%d %s dam %02X %s %08x",t,s,b[0],b[1],b[2],b[3],
          idok?"ok":"BAD",dam&0xff,dam<0?"-":(datok?"ok":"BAD"),dam<0?0:h);
        lines.push_back(ln);
      }
      std::sort(lines.begin(),lines.end()); lines.erase(std::unique(lines.begin(),lines.end()),lines.end());
      for(auto&l:lines) puts(l.c_str());
    }
    im.Close();
  }
  return 0;
}
