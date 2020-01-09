#define ZERYNTH_PRINTF
#include "zerynth.h"
#include "bme280.h"

int bmedigs[18];
int bmedrv;
int bmeaddr;
int bmetimeout=1000;

C_NATIVE(_bme280_setup){
    NATIVE_UNWARN();

    if (nargs<3) return ERR_TYPE_EXC;
    PList *ps = (PList*) args[0];
    int i;
    for(i=0;i<18;i++) {
        bmedigs[i] = PSMALLINT_VALUE(PLIST_ITEM(ps,i));
    }
    bmedrv = PSMALLINT_VALUE(args[1]);
    bmeaddr = PSMALLINT_VALUE(args[2]);
    return ERR_OK;
}

C_NATIVE(_bme280_getfast){
    NATIVE_UNWARN();

    RELEASE_GIL();
    uint32_t temp, hum,press;
    bme280_acquire(&temp,&hum,&press);
    PTuple *tpl = ptuple_new(3,NULL);
    PTUPLE_SET_ITEM(tpl,0,pinteger_new(temp));
    PTUPLE_SET_ITEM(tpl,1,pinteger_new(hum));
    PTUPLE_SET_ITEM(tpl,2,pinteger_new(press));
    *res = tpl;


    ACQUIRE_GIL();
    return ERR_OK;
}
int t_fine;
int bme280_convert_temp(uint8_t *raw, int *digs){
    int adc_t = (raw[0]<<16) | (raw[1] <<8) | raw[2];
    int var1,var2,T;
    adc_t = adc_t>>4;
    var1 = (((adc_t>>3) - (digs[0]<<1))*digs[1])>>11;
    var2 = (((((adc_t>>4) - (digs[0])) * ((adc_t>>4) - (digs[1]))) >> 12) * (digs[2])) >> 14;
    t_fine = var1+var2;
    T = (t_fine*5+128)>>8;
    return T;
}
int bme280_convert_press(uint8_t *raw, int *digs){
    int adc_p = (raw[0]<<16) | (raw[1] <<8) | raw[2];
    int var1,var2;
    adc_p = adc_p>>4;
    if (adc_p == 0x80000) return 0;

    var1 = (t_fine >> 1) - 64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * digs[5];
    var2 = var2 + ((var1*digs[4])<<1);
    var2 = (var2>>2)+(digs[3]<<16);

    var1 = (((digs[2] * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((digs[1] * var1)>>1))>>18;
    var1 = ((32768+var1)*(digs[0]))>>15;

    if (var1 == 0) return 0;

    int p = (((1048576-adc_p)-(var2>>12)))*3125;
    p = (p / var1 ) << 1;

    var1 = (digs[8] * (((p>>3) * (p>>3))>>13)) >> 12;
    var2 = ((p>>2) * digs[7]) >> 13;

    p = p + ((var1 + var2 + digs[6]) >> 4);

    return p;
}

uint32_t bme280_convert_hum(uint8_t *raw, int *digs){
    int adc_h = (raw[0]<<8) | raw[1];
    if (adc_h == 0x8000) return 0;

    uint32_t v_x1_u32r = t_fine - 76800;
    v_x1_u32r = (((((adc_h << 14) - (digs[3] << 20) -
                    (digs[4] * v_x1_u32r)) + 16384) >> 15) *
                 (((((((v_x1_u32r * digs[5]) >> 10) *
                      (((v_x1_u32r * digs[2]) >> 11) + 32768)) >> 10) +
                    2097152) * digs[1] + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               digs[0]) >> 4));

    if (v_x1_u32r<0) v_x1_u32r=0;
    if (v_x1_u32r>419430400) v_x1_u32r=419430400;

    uint32_t h = (v_x1_u32r>>12);
    //return fixed point  xx.yyyy  converting from 1/102 to 1/1000
    return ((h>>10)*1000)+(((h&0x3ff)*1000)>>10);
}

int bme280_acquire(uint32_t* temp, uint32_t *hum, uint32_t *press){
    uint8_t tbuf[8];
    uint8_t treg=0xF7;
    vhalI2CSetAddr(bmedrv,(uint16_t)bmeaddr);
    if(vhalI2CTransmit(bmedrv,&treg,1,tbuf,8,TIME_U(bmetimeout,MILLIS))==0) {
        //result is tbuf =  p p p t t t h h
        *temp  = bme280_convert_temp(tbuf+3,(int*)bmedigs);
        *press = bme280_convert_press(tbuf,((int*)bmedigs)+3);
        *hum   = bme280_convert_hum(tbuf+6,((int*)bmedigs)+12);
        return 0;
    }
    return -1;
}
