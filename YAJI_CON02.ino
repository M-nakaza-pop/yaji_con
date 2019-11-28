/********************************************************************************************************************
*  SD card read/write                                                                                               *
*  SparkFun SD Card Shield                                                                                                                 *
* This example shows how to read and write data to and from an SD card file                                         *
* The circuit:                                                                                                      *
* * SD card attached to SPI bus as follows:                                                                         *
* ** MOSI - pin 11                                                                                                  *
* ** MISO - pin 12                                                                                                  *
* ** CLK - pin 13                                                                                                   *
* ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)                                                                     *
********************************************************************************************************************/
#include    <MsTimer2.h>

#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
/***********************************************************************************************************************/
/*  
/***********************************************************************************************************************/
#define     ldpin       6
#define     clkpin      5
#define     inputpin    3           //* dammy *

#define     latchpin    7           //* PD7 *
#define     clockpin    5           //* PD5 *
#define     outputpin   4           //* PD4 *

#define     DE          8           /* RS485 DE */

#define     DS0         14          //A0 PC0
#define     DS1         15          //A1 PC1
#define     DS2         16          //A2 PC2
#define     DS3         17          //A3 PC3



#define     STX         0x02        //* Start of Text *
#define     ENQ         0x05        //* Enquiry *
#define     ETX         0x03        //* End of Text *

#define     indexEND    370

#define     tab         11

#define     RXDMAX      660
#define     SETLENGTH   9           // 47 30 31 30 30 30 30 30 30 0d 0a
#define     OUTTABLE    16
#define     RMAX        60


//          PINB    0x03
//          DDRB    0x04
//          PORTB   0x05

//          PINC    0x06
//          DDRC    0x07
//          PORTC   0x08

//          PIND    0x09
//          DDRD    0x0A
//          PORTD   0x0B

/***********************************************************************************************************************/
/* Global variables and functions 
/***********************************************************************************************************************/
// Chip Select pin is tied to pin 8 on the SparkFun SD Card Shield

const int chipSelect = 10;  


byte    data[indexEND];             // rxdBff
int     g_index     =   0;          // bffCount
byte    rxFlg       =   0;
boolean commY       =   true;       //false;(本番）

byte    rnum[RMAX]  =   { 0 };      //* 0 - 59 *

volatile    int     count1      =   0;
volatile    int     count2      =   1800;

byte    buff[11];                   //* 47 30 31 30 30 30 30 30 30 0d 0a *
byte    outp[16]    =   { 0 };
byte    temp[16];

byte    dipaddr = 0;
byte    dipmode = 0;

unsigned int seridata   = 0;
/*
int     outputpin       =   2;      //* PD2 *
int     clockpin        =   5;      //* PD5 *
int     latchpin        =   7;      //* PD7 *
*/

/***********************************************************************************************************************/
/*   Prototype
/***********************************************************************************************************************/
void    convData(byte *dest, byte *addr, byte lim);
void    inputArray(byte *buff);
void    iniSpi();
void    iniData();
void    clkWait();
void    allOr(byte *dest, byte  *addr, byte lim);
byte    dip8Read();
byte    dip4Read();
void    startFa();
void    timerFire();
void    outSerial(int *sdata, byte *addr );
byte    ascConv(byte numb);
byte    kUnfold(byte *dest, byte *addr);
void    dataClr(byte *dest, byte lim);
void    dataSet(byte *dest, int num);
void    popEep(byte flg);
long    hc165Read(byte count);

void    testMode();
void    testMoni1(byte *addr);
void    buffMoni(byte *rnum, byte lim);
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void setup(){
    
    Serial.begin(19200);
    
    while (!Serial) {
        Serial.println(F("initialization Serial failed!"));    //flash-> RAMを消費しない  
    }
        
    if (!SD.begin(4)) {
        Serial.println(F("initialization SDcard failed!"));    //flash-> RAMを消費しない
    }

    pinMode(DS0,INPUT);
    pinMode(DS1,INPUT);
    pinMode(DS2,INPUT);
    pinMode(DS3,INPUT);
   
    pinMode(9, OUTPUT); 
    pinMode(DE, OUTPUT);
    
    pinMode(latchpin, OUTPUT);
    pinMode(clockpin, OUTPUT);
    pinMode(outputpin, OUTPUT);

    pinMode(inputpin,INPUT);                      /* seri input */
    pinMode(ldpin,OUTPUT);                        /* L-LD/H-SHIFT */
    pinMode(clkpin,OUTPUT);
    
    digitalWrite(DE, HIGH);
    digitalWrite(latchpin, LOW);
    digitalWrite(clockpin, LOW);
    digitalWrite(outputpin, LOW);
 
    MsTimer2::set(50, timerFire);    //* 50mSEC *
    MsTimer2::start();

    iniSpi();
    iniData();                      //* SD Card -> EEPROM *　
    
    dataClr(outp, OUTTABLE);         // OUTTABLE = 16
    outSerial(&seridata, outp);
    
    dipaddr =   dip8Read();
    dipmode =   dip4Read();
    testMode();
    startFa();
    
    Serial.print("MAIN");
    Serial.println(); 
}   
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/  
void loop(){
    
        serRead();
        stxComm();                      // 内部で-> popEep()を呼ぶ
        outSerial(&seridata, outp);     // OUTTABLE = 16
        //commY = lifeTime(&count2);    // lifeTimeを失うと参加資格も失う
}
/***********************************************************************************************************************
* Function Name: 
* Description  : バッファリングのみ、処理はenqComm()かstxComm()で
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void serRead(){

    if(Serial.available() >0 ){                     //-1が返ることもある
  
        data[g_index]   =   Serial.read();
        if(rxFlg    == 0x03){                       //* 03受信後の+LRCで完了
            rxFlg   ='!';                           //* 0x21 
        }
        if(data[g_index]    ==STX && rxFlg==0){     /*STX　02受信*/
            rxFlg   =0x02;
            g_index =0;
            data[g_index]   =0x02;
        }
        if(data[g_index]    ==ETX && rxFlg==0x02){  /*ETX　03受信*/
            rxFlg=0x03;
        }
//        if(data[g_index]    ==ENQ && rxFlg==0)    /*ENQ*/
//        {
//            rxFlg   ='?';                         /* 0x3F */
//            g_index    =0;
//            data[g_index]   =0x05;                /* ENQ */
//        }
        g_index++;
        if(g_index  >=  indexEND){
            g_index=0;
        }      
    }
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void stxComm(){
    
    byte    ans;
    
    if(rxFlg=='!'){
        
        if(ascConv(data[2])!=(dipaddr & 0x0f) || 'D'!=data[1]){
            rxFlg = 0;
            return;    
        }
        rxFlg   = 0;
        
        if(data[3]=='Y'){                   /*Y電文*/           
            commY   =   true;
            count2  =   1800;

            digitalWrite(DE, LOW);
            Serial.write(0x02);
            Serial.print(F("FFi"));
            Serial.write(0x03);
            Serial.write(0x6A);
            Serial.flush();
            digitalWrite(DE, HIGH);
            
            return;        
        }

        count2  = 1800;                     //* life counter
                    
        if(data[3]=='V' && commY==true){    /*V電文*/    
            String  buffv    =   "";
            buffv    +=  "FFx00";
            buffv    +=  0x00;              /* null */
            digitalWrite(DE, LOW);
            Serial.write(0x02);
            Serial.print(buffv);            /* String なら OK */
            Serial.write(0x03);
            Serial.write(0x7B);
            Serial.flush();
            digitalWrite(DE, HIGH);
        }
        
        if(data[3]=='K' && commY==true){
            ans =   kUnfold(rnum,data);     // rnum[RMAX] data[indexEND]
            digitalWrite(DE, LOW);
            Serial.write(ans);              // ACK or NAK            
            Serial.flush();
            digitalWrite(DE, HIGH);
            
            if(ans == 0x06){
                popEep(rnum);          
            }
        }                         
    }
}
/***********************************************************************************************************************
* Function Name: K_Unfold(k電文_展開)
* Description  : dara[] -> rnum[]
* Arguments    : data[0]
* Return Value : return(0x06)
***********************************************************************************************************************/
byte    kUnfold(byte *dest, byte *addr){

    int     i   =   3;
    byte    a   =   0;                      //addr
    byte    s   =   0;                      //status
    
    for(int l = 0; l<RMAX; l++){
        
        if(addr[i] == 'K'){
            a   = (ascConv(addr[++i])<<4);
            a   = a | ascConv(addr[++i]);     // 2byte -> HEX
            
            if(a > 0x3B){
                return(0x15);    
            }
            s   =   ascConv(addr[++i]);       //status -> HEX 0x00,0x01,0x02
            if(s > 0x02){
                return(0x15); 
            }
            i++;
            dest[a] = s;                    //rmun
        }
        if(addr[i] == 0x03){
            return(0x06);        
        }
    }
    return(0x15);
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : ans 0x06 or 0x15
* Return Value : None
***********************************************************************************************************************/
void    popEep(byte *addr){

    dataClr(outp, OUTTABLE);             //OUTTABLE = 16
    
    for(byte i=0; i<RMAX; i++){
        
        if(addr[i] != 0){               //rnum[]
            
            dataSet(buff, i );          //* EEP -> buff[0]-buff[8]
            inputArray(buff);           //* buff -> buff
            convData(temp, buff+1, 8);  //* buff[8] -> temp[16]  G抜きの為 buff+1            
            allOr(outp, temp, OUTTABLE);//* temp[16] -> outp[16]
        
        }
    }
}
/***********************************************************************************************************************
* Function Name: 
* Description  : outp[] から seridataの作成してseridataをhc595へ出力
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void    outSerial(int *sdata, byte *addr ){

        if(count1   !=  0){
            return;
        }
        
        count1  =   8;                              /*50m*8＝0.4SEC */        
        //unsigned int seridata   = 0;              //これだと点滅しないよ
        unsigned int mask       = 0x0001;
        
        for(byte index =0; index <15; index++){               /* ASCのOUTは 0-15 */           
            //rockRetry(index);
            switch(addr[index]){
                case    0x00:
                    *sdata    =   *sdata    &  ~mask;
                    break;
                case    0x01:
                    *sdata    =   *sdata    |   mask;
                    break;
                case    0x02:                                       /* BLINK */
                    *sdata    =   *sdata    ^   mask;           /* EOR */
                    break;     
                case    0x03:
                    *sdata    =   *sdata    ^   mask;           /* 3=EOR */
                    //seridata    =   seridata    &   ~mask;          /* ONE SHOT CLR */ 
                    //addr[index] =   0x00;
                    break;

                default:
                    *sdata    =   *sdata    &  ~mask;           /*0,1,2,3以外は0 */
                    //seridata    =   seridata    |   mask;     /* one shot */
                    //addr[index]--;                            /* 0x05->0x04->0x03 この間0.8SEC */  
            }
            mask  =mask    << 1;  
        }

        shiftOut(outputpin, clockpin, MSBFIRST, (byte)*sdata);            //SPI.transfer(seridata); 
        shiftOut(outputpin, clockpin, MSBFIRST, (byte)(*sdata>>8));
        clkWait();
        digitalWrite(latchpin, HIGH);
        digitalWrite(latchpin, LOW);
}
/***********************************************************************************************************************
* Function Name: 
* Description  : SD Card -> EEPROM
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void    iniData(){                          /* 47 30 31 30 30 30 30 30 30 0d 0a */

    byte    a;
    File    myFile;                         /* 32byte pointer */
    myFile = SD.open("output.txt");

    if (myFile){                           //Serial.println("output.txt:");  

        for(int i=0; i<RXDMAX; i++){
            
            a   =   myFile.read();
            //EEPROM.write(i, a);           /* 3.3mS*660=2178mSEC */   
            EEPROM.update(i, a);            // 更新時のみ書変え
        }      
    }else{
        Serial.println("error opening output.txt"); //error:
        error();
    } 
    myFile.close(); 
}
/***********************************************************************************************************************
* Function Name: 
* Description  : EEPROM -> SRAM
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void    dataSet(byte *dest, int num ){      

    int a = tab * num;                          //0d 0aは抜くので連続しない
    
    for(byte i=0; i< SETLENGTH; i++){
        dest[i] =   EEPROM.read(a);
        a++;
    }    
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 0 -> dest[i]
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void    dataClr(byte *dest, byte lim){
    
    for(byte i=0; i< lim; i++){
        dest[i] =   0;
    }
    
}
/***********************************************************************************************************************
* Function Name: * conversion data Multiplex data *
* Description  : buff[8]->temp[16]
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void    convData(byte *dest, byte *addr, byte lim){
      
        while(*addr!=0x0d && lim){                  //* 47 30 31 30 30 30 30 30 30 0d 0a *
            *dest++ = (*addr >> 2) & 0x03;
            *dest++ = *addr++ & 0x03;
            lim--;           
        }
}
/***********************************************************************************************************************
* Function Name: ArrayChange
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void    inputArray(byte *addr){
        
        byte    a[5];               //[0]はダミー

        a[1]     =addr[1];           /* addr[0]=G skip */
        a[2]     =addr[2];
        a[3]     =addr[3]; 
        a[4]     =addr[4];

        addr[1]     =a[2];
        addr[2]     =a[1];
        addr[3]     =a[4];
        addr[4]     =a[3];      
        
        a[1]     =addr[5];
        a[2]     =addr[6];
        a[3]     =addr[7];
        a[4]     =addr[8];

        addr[5]     =a[6];
        addr[6]     =a[5];
        addr[7]     =a[8];
        addr[8]     =a[7];
                     
}
/***********************************************************************************************************************
* Function Name: outp[16] or temp[16]
* Description  : 
* Arguments    : (byte *dest ,byte  *addr ,byte lim)
* Return Value : None
***********************************************************************************************************************/
void    allOr(byte *dest, byte  *addr, byte lim){
        
    for(byte i=0; i<lim; i++){
        dest[i] =   dest[i] | addr[i];
    }   
}
/***********************************************************************************************************************
* Function Name: TestMoni
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void    rnumMoni(){
    
    for(byte i=0; i<RMAX; i++){
        Serial.write(rnum[i]);
    }    
}
/***********************************************************************************************************************
* Function Name: TestMoni
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void    buffMoni(byte *addr, int limit){
    
    for(byte i=0; i<limit; i++){
        Serial.write(addr[i]);
    }    
}
/***********************************************************************************************************************
* Function Name: 
* Description  : HC595 ALL 0
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void    iniSpi(){
    
    shiftOut(outputpin, clockpin, MSBFIRST, 0x00);            //SPI.transfer(seridata); 
    shiftOut(outputpin, clockpin, MSBFIRST, 0x00);
    clkWait();                              // 1uSのNOP（）
    digitalWrite(latchpin, HIGH);           // 3uSのパルス幅になる
    digitalWrite(latchpin, LOW);
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 0.0625us * 16 = 1uS
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void    clkWait(){
         asm volatile(      
            
            "nop \n"            //0.0625us
            "nop \n"            //0.0625us
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
        );
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
byte    dip8Read(){
    
        unsigned long    dip8    = 0x0000;
        
        dip8    = hc165Read(24);        
        //Serial.print((byte)dip8);
        //Serial.println();
        return((byte)dip8);       
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
byte    dip4Read(){
        
        byte    mode;
        mode = (~PINC) & 0x0F;
        return(mode);
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 割り込み処理から呼ばれる
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void timerFire(){

    if(count1   !=  0){
        count1--    ;
    }
    if(count2   !=  0){                             /* LIFE COUNT */
        count2--    ;
    }
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
long     hc165Read(byte count){

        long     hc165data  =0;
        byte    i = count - 1;
        
        digitalWrite(clkpin,LOW);
        digitalWrite(ldpin,LOW);
        clkWait();
        digitalWrite(ldpin,HIGH);
        clkWait();
        if(digitalRead(inputpin)==HIGH){
            hc165data  |=0x0001;
        }
        hc165data    = hc165data << 1;
        
        clkWait();
        
        for(byte loop165=0; loop165 < i ;loop165++)     //14
        {    
            digitalWrite(clkpin,HIGH);
            clkWait();
            digitalWrite(clkpin,LOW);
            
            if(digitalRead(inputpin)==HIGH){
                hc165data  |=0x0001;
            }
            hc165data    = hc165data << 1;
            clkWait();    
        }
        return(~(hc165data >>1));                   /* adjst */
}
/***********************************************************************************************************************
* Function Name: 
* Description  : ascii -> HEX 1byte
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
byte    ascConv(byte numb){
       
        if (numb < '0' || numb > 'F'){
            return(0x7F);                   /* ERR */        
        }        
        if (numb    <= '9'){
            return(numb &0x0F);             /* 00-09 */
        }
        if (numb    <= 'F'){                /* 0x46 */
            return(numb -0x37);             /* 0A-0F */
        }                  
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
boolean lifeTime(int *addr){
            if(*addr){
                return(true);        
            }else{
                dataClr(rnum, RMAX);                // 60
                dataClr(outp, OUTTABLE);            // 16
                seridata   = 0;
                return(false);
            }
        }
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
struct  convnum{
        byte anum ;
        byte hnum ;
}

const   convnum[16] ={  '0',0x00,
                        '1',0x01,
                        '2',0x02,
                        '3',0x03,
                        '4',0x04,
                        '5',0x05,
                        '6',0x01,
                        '7',0x02,
                        '8',0x03,
                        '9',0x04,
                        'A',0x05,
                        'B',0x01,
                        'C',0x02,
                        'D',0x03,
                        'E',0x04,
                        'F',0x05    };
/***********************************************************************************************************************
* Function Name: 
* Description  : ASCI -> HEX  1byte
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
byte     select(byte anum){
        int hnum    = 0x7F;                     /*ERR CODE*/

        for(byte i=0; i<16; i++){
            if(convnum[i].anum   ==  anum){
                hnum = convnum[i].hnum;
                break;
            }
        }
        return(hnum);
}
/***********************************************************************************************************************
* Function Name: TEST MODE
* Description  : 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void    testMode()
{
    long    txdwait =   2000;
    byte    index   =   0;
    byte    buffer[6]   ;
    
        
        while(dipmode & 0b00000001){
            
            if(count1  ==  0){
                
                count1  =   8;
                int tempdata    =((int)hc165Read(16) & 0x3FFF);
#if 0                
                if( remrecv==true && remin.data.code1 == 0x3581){                   /* 3581 */
                    bitRead(tempdata,14)==0 ? bitSet(tempdata,14): bitClear(tempdata,14);
                    remrecv  = false; 
                }
#endif                
                digitalWrite(clockpin, LOW);            //実行前にlowにする 
                shiftOut(outputpin, clockpin, MSBFIRST, (byte)tempdata);            //SPI.transfer(seridata); 
                shiftOut(outputpin, clockpin, MSBFIRST, (byte)(tempdata >>8));
                clkWait();
                digitalWrite(latchpin, HIGH);
                digitalWrite(latchpin, LOW);

            }

            if(--txdwait  ==  0){
                    index   =   0;
                    txdwait =   400000;
                    digitalWrite(DE, LOW);
                    Serial.print("test");
                    Serial.flush();
                    digitalWrite(DE, HIGH);
                }

            
            if(Serial.available() > 0){
                buffer[index] = Serial.read();
                index++ ;
            }
            if(index    ==  4){
                 buffer[index] = '\0';
                 String s   =  buffer ;
                    //Serial.print(s);
                    index   =   0;
                 if(s.equals("test")){
                    //Serial.print("ok");
                    txdwait =   2000;
                 }
            }     
            clkWait();   
        }
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void    startFa(){
    
    Serial.print("SFA");
    Serial.println(); 
    
        while(true){
            if (0xFA==Serial.read()){
                return;
            } 
            if ( 0x05==Serial.read()){
                digitalWrite(DE, LOW);
                for(byte num=0  ;num    <30 ; num++){
                    Serial.write(0xFA);    
                }
                Serial.flush();
                digitalWrite(DE, HIGH);
                return;
            }
        }
}
/***********************************************************************************************************************
* Function Name: 
* Description  : error opening output
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void    error(){
    
        boolean al = true;
        int     tempdata = 0;
    
        while(true){
            if(count1  ==  0){
                count1 =  8 ;                
                
                if(al == true){
                    al = false;
                    digitalWrite(9,LOW);
                    bitSet(tempdata,14);
                }else{
                    al = true;
                    digitalWrite(9,HIGH);
                    bitClear(tempdata,14);
                    }
                    
                digitalWrite(clockpin, LOW);            //実行前にlowにする 
                shiftOut(outputpin, clockpin, MSBFIRST, (byte)tempdata);            //SPI.transfer(seridata); 
                shiftOut(outputpin, clockpin, MSBFIRST, (byte)(tempdata >>8));
    
                clkWait();
                digitalWrite(latchpin, HIGH);
                digitalWrite(latchpin, LOW);              
            }   
        }
}

                        
