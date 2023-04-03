#include "headfile.h"
#include "encoder.h"
#include "motor.h"
#include "timer_pit.h"
#include "display.h"




void display_entry(void *parameter)
{
    while(1)
    {
        ips114_displayimage032_zoom(mt9v03x_image[0], MT9V03X_W, MT9V03X_H, IPS114_X_MAX, IPS114_Y_MAX);
        ips114_showint32(1,0, encoder_data[0],5);
        ips114_showint32(1,1, encoder_data[1],5);
        ips114_showint32(1,2, encoder_data[2],5);
        ips114_showint32(1,3, encoder_data[3],5);

    }
}





void display_init(void)
{
    rt_thread_t tid1;

    //初始化屏幕
    ips114_init();
    
    //创建显示线程 优先级设置为6 
    tid1 = rt_thread_create("display", display_entry, RT_NULL, 1024, 6, 50);
    
    //启动显示线程
    if(RT_NULL != tid1)
    {
        rt_thread_startup(tid1);
    }
}
