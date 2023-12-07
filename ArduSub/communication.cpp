#include "Sub.h"

void Sub::send_to_rasp(float* error,int mode,int isarm)
{
    int i;
    _buffertx[0] = Frame_Header1;    //head1
    _buffertx[1] = Frame_Header2;    //head2

    _buffertx[protocol_len] = 12; //len is 12
    _buffertx[4] = isarm;
    _buffertx[3] = mode;

    x_error.value = error[0];
    y_error.value = error[1];
    yaw_error.value = error[2];
    
    for(i = 5; i < 9; i++){
        _buffertx[i] = x_error.value_char[i - 5];
    }
    
    for(i = 9; i < 13; i++){
        _buffertx[i] = y_error.value_char[i - 9];
    }

    for(i = 13; i < 17; i++){
        _buffertx[i] = yaw_error.value_char[i - 13];
    }

    _buffertx[17] = Frame_Tail1;
    _buffertx[18] = Frame_Tail2;  

    //for(i = 0;i < 19;i++){
        hal.uartD->write(_buffertx,19);
    //}
    _buffertx[0] = 0x00;
    _buffertx[1] = 0x00;
    _buffertx[2] = 0x00;
    _buffertx[3] = 0x00;
    _buffertx[4] = 0x00;
    _buffertx[5] = 0x00;
    _buffertx[6] = 0x00;
    _buffertx[7] = 0x00;
    _buffertx[8] = 0x00;
    _buffertx[9] = 0x00;
    _buffertx[10] = 0x00;
    _buffertx[11] = 0x00;
    _buffertx[12] = 0x00;
    _buffertx[13] = 0x00;
    _buffertx[14] = 0x00;
    _buffertx[15] = 0x00;
    _buffertx[16] = 0x00;
    _buffertx[17] = 0x00;
    _buffertx[18] = 0x00;
}


void Sub::receive_from_rasp()
{
    f_h1_flag = 0;
    f_h_flag = 0;
    f_t1_flag = 0;
    bool pass_flag = false;
    uint32_t numc = 0;
    int16_t len = 0;
    int16_t payload = 5;
    int16_t mode = 3;

    numc = hal.uartD->available();
    uint32_t tnum = 0;
    if ( numc == 19){
        pass_flag = true;
        //hal.uartC->printf("receive_form_rasp");
    }
    else{
        pass_flag = false;
        temp_pass_flag = numc;        
    }
    
    //uint32_t run_time;
    //run_time = AP_HAL::micros() - sensor_start_time;//now - start

    if(pass_flag)
    {
        while(numc--)
        {
            _bufferrx[tnum] = hal.uartD->read();

            if(f_h_flag == 1)
            {
                if(f_t1_flag == 1)
                {
                    if (_bufferrx[tnum] == Frame_Tail2)
                    {
                        //hal.uartC->printf("receive_form_rasp");
                        temp_pass_flag = 0;
                        len = _bufferrx[2];
                        if((_bufferrx[mode] & 0x79) == 0x79){                        
                            
                            int i = 0;
                            for(i = 5; i < payload + (len/3); i++){
                                x_force.value_char[i - 5] = _bufferrx[i];
                            }
                            asv_x_robust_force = x_force.value;
                            //hal.uartC->printf("asv_x_robust_force:%f\n",asv_x_robust_force);
                            for(i = 9; i < payload + (len/3*2); i++){
                                y_force.value_char[i - 9] = _bufferrx[i];
                            }
                            asv_y_robust_force = y_force.value;
                            
                            for(i = 13; i < payload + len; i++){
                                yaw_force.value_char[i - 13] = _bufferrx[i];
                            }
                            asv_yaw_robust_force = yaw_force.value;
                        } 
                        tnum = 0;
                    }else
                    {
                        f_t1_flag = 0;
                        tnum++;
                    }
                }else
                {
                    if (_bufferrx[tnum] == Frame_Tail1)
                    {
                        f_t1_flag = 1;
                        tnum++;
                    }
                    else
                    {
                        tnum++;
                    }
                }
            }
            else{
                if (f_h1_flag == 1)
                {
                    if (_bufferrx[tnum] == Frame_Header2)
                    {
                        f_h_flag = 1;
                        tnum++;
                    }
                    else
                    {
                        f_h1_flag = 0;
                        tnum = 0;
                    }
                }
                else
                {
                    if (_bufferrx[tnum] == Frame_Header1)
                    {
                        f_h1_flag = 1;
                        tnum++;
                    }
                    else
                    {
                        tnum = 0;
                    }
                }
            }
        }
    }
    else
    {
        while(numc--){
        hal.uartD->read();
        }
    }
    _bufferrx[0] = 0x00;
    _bufferrx[1] = 0x00;
    _bufferrx[2] = 0x00;
    _bufferrx[3] = 0x00;
    _bufferrx[4] = 0x00;
    _bufferrx[5] = 0x00;
    _bufferrx[6] = 0x00;
    _bufferrx[7] = 0x00;
    _bufferrx[8] = 0x00;
    _bufferrx[9] = 0x00;
    _bufferrx[10] = 0x00;
    _bufferrx[11] = 0x00;
    _bufferrx[12] = 0x00;
    _bufferrx[13] = 0x00;
    _bufferrx[14] = 0x00;
    _bufferrx[15] = 0x00;
    _bufferrx[16] = 0x00;
    _bufferrx[17] = 0x00;
    _bufferrx[18] = 0x00;
}