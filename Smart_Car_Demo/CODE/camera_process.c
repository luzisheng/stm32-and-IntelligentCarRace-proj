#include "camera_process.h"
#include "fork.h"
#include "buzzer.h"
#include "motor.h"
#include "headfile.h"

uint8 leftBorder[MT9V03X_H];
uint8 rightBorder[MT9V03X_H];
uint8 midLine[MT9V03X_H];

/*环岛标志位下屏蔽三岔*/
uint8 circle_flag1 = 0;  //环岛突然丢线标志
uint8 circle_flag2 = 0;  //环岛看到出环岛标志，经陀螺仪积分后退出标志！
uint8 circle2_get_raw = 0;

uint8 garage_get_flag = 0; //寻到车库标志
uint8 garage_circle_differ = 0; //车库和环岛区别
uint8 stop_flag = 0; //停车标志

uint8 fork_flag = 0;  //无丢线普通赛道为1，三岔路为2
uint8 ex_fork_flag = 0;

//边界处理,寻找中线
void getborder(uint8 (*img)[MT9V03X_W])
{
    uint8 temp_w = 0; //暂量
    uint8 temp_h = 0;
    uint8* img_ptr = img[0];
    uint8 road_mid = 94; //值为MT9V03X_W / 2; 首次寻中线从中间开始,之后考虑中线连续,可据此增加其余判断
    uint8 road_mid_new; //与上一行中线比较避免中线突变

    uint8 circle_flag_temp = 0;  //环岛暂时判断标志

    uint8 left_lose_flag = 0;  //左边丢线置1，重新寻到线置2
    uint8 right_lose_flag = 0; //右边丢线置1，重新寻到线置2
    uint8 left_border_flag = 0;//左边找到边界标志
    uint8 right_border_flag = 0;//右边找到边界标志
    uint8 fork_road_flag = 0; //三岔路标志
    uint8 fork_road_raw = 0;  //三岔路分叉行
    uint8 left_lose_raw = 0;   //左边开始丢线行
    uint8 left_get_raw = 0;    //左边重新找到线的行
    uint8 right_lose_raw = 0;  //右边开始丢线行
    uint8 right_get_raw = 0;    //左边重新找到线的行

    uint8 garage_mark = 0; //车库黑白跳变标志

    uint8 i = 0;//test

    for (temp_h = 49 ; temp_h > 0; temp_h--)  //寻找50行图像中线,for循环内不论是否丢线
    {
        if(temp_h > 25) //判断三岔路
        {
            img_ptr = img[temp_h - 5];
            if( road_mid > 70 &&  img_ptr[road_mid] < THRESHOLD && img_ptr[road_mid-1] < THRESHOLD && !circle_flag1 && !circle_flag2)//上一行的中线正前方5行出现赛道边界！可能为三岔路,并且去除误判！！！！
            {
                 fork_road_flag = 1;
                 fork_road_raw  = temp_h;
            }
        }
        img_ptr = img[temp_h];
        left_border_flag = 0;
        right_border_flag = 0;

        //开始搜线
        for (temp_w = road_mid; temp_w < MT9V03X_W; temp_w++)
        {
            if (img_ptr[temp_w] < THRESHOLD && img_ptr[temp_w - 1] < THRESHOLD)
            {
                rightBorder[temp_h] = temp_w - 2;
                img_ptr[temp_w - 2] = 0; //显示右边界
                img_ptr[temp_w - 3] = 0; //显示右边界
                img_ptr[temp_w - 4] = 0; //显示右边界
                right_border_flag = 1;
                break;
            }
        }
        if(right_border_flag == 0)//没搜到右边界
        {
            rightBorder[temp_h] = 186;
            img_ptr[186] = 0; //显示右边界
            img_ptr[185] = 0; //显示右边界
            img_ptr[184] = 0; //显示右边界

            if(right_lose_flag == 0)//首次没搜到右边界
            {
                right_lose_flag = 1;     //右边界丢失
                right_lose_raw = temp_h; //记录首次丢线的行
            }
        }

        if(right_border_flag == 1 && right_lose_flag == 1)  //丢线后重新寻到
        {
            right_lose_flag = 2;  //边界丢失又找回
            right_get_raw = temp_h; //重新找回线位置
        }

        for (temp_w = road_mid; temp_w > 0; temp_w--)
        {
            if (img_ptr[temp_w] < THRESHOLD && img_ptr[temp_w + 1] < THRESHOLD)
            {
                leftBorder[temp_h] = temp_w + 2;
                img_ptr[temp_w + 2] = 0; //显示左边界
                img_ptr[temp_w + 3] = 0; //显示左边界
                img_ptr[temp_w + 4] = 0; //显示左边界
                left_border_flag = 1;
                break;
            }
        }
        if(left_border_flag == 0)//没搜到左边界
        {
            leftBorder[temp_h] = 2;
            img_ptr[2] = 0; //显示左边界
            img_ptr[3] = 0; //显示左边界
            img_ptr[4] = 0; //显示左边界

            if(left_lose_flag == 0)//没搜到左边界的第一行
            {
                left_lose_flag = 1;     //左边界丢失
                left_lose_raw = temp_h; //丢线位置
            }
        }

        if(left_border_flag == 1 && left_lose_flag == 1)  //丢线后重新寻到
        {
            left_lose_flag = 2;  //边界丢失又找回
            left_get_raw = temp_h; //重新找回线位置
        }

    /*
     *  if(rightBorder[temp_h] - leftBorder[temp_h] < 2)//大弯赛道从侧方拐出，最前方一片黑，这种情况不应计算全黑的边界
        {
            road_mid = 94;
        }
    */

        else{
        road_mid_new = (leftBorder[temp_h] + rightBorder[temp_h]);//如果丢线，赛道边界为图像边界，在后面矫正
        road_mid_new >>= 1;   //执行除以二

        if(road_mid_new - road_mid >15) road_mid = road_mid_new - 15;//中线限幅
        else if(road_mid_new - road_mid < -15) road_mid = road_mid_new + 15;
        else road_mid = road_mid_new;
        }

        if(img_ptr[road_mid] == 0)//特指突然中线跑到图像最左边还挺稳定的奇葩情况，中线跑到黑线上了
        {
            road_mid = midLine[temp_h - 2];//中线回去！！！
        }
        midLine[temp_h] = road_mid; //中线
        img_ptr[road_mid] = 0;  //将处理得到的中线变为黑色
    }

    /*发生丢线时情况处理*/


    /*环岛标志2特有*/
    if ( circle_flag1 == 1 && left_lose_flag == 2 )//在标志1下 左边开始丢线，后来寻到,去除右边不丢线的判断
    {
        if( leftBorder[left_get_raw] > 30) //属于突然寻到
         {
            switch(circle_flag2){
            case 0:
                circle_flag2 = 1;         //识别到环岛！！！！！！！！！！！！
                //rt_mb_send(buzzer_mailbox, 300);//蜂鸣器发出声音！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
                break;
            case 4:
                circle_flag2 = 5;         //完全直行出环岛
                //rt_mb_send(buzzer_mailbox, 300);//蜂鸣器发出声音！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
                break;
            default:
                break;
            }
            //circle_flag1 = 0;  //环岛标志位在出环岛前一直不清0
         }
    }

    if(circle_flag2 == 5)
    {
        circle2_get_raw = left_get_raw; //此时出环岛看到标志位2只计算找到左边线后左右都有的一部分
    }

    ex_fork_flag = fork_flag;

    /*无丢线，认为普通赛道*/
    if(left_lose_flag == 0 && right_lose_flag == 0)
    {
        fork_flag = 1;
        garage_circle_differ = 0;//解锁车库和环岛的互锁
        //rt_event_send(fork_event, EVENT_FLAG3);

        if(circle_flag2 == 5)  //环岛之后，普通赛道退出环岛
        {
            circle_flag1 = 0;
            circle_flag2 = 0;
            /*出环岛加速*/
//            set_speed = 350;
//            kp = 65;
        }
    }

    /*只有 左边丢线 判断车库 和 环岛(排除十字影响)*/
    else if(left_lose_flag > 0 && right_lose_flag == 0)
    {
        for(temp_h = left_lose_raw; temp_h > 0; temp_h--)
        {
            garage_mark = 0;
            img_ptr = img[temp_h];
            for(i=0;i<135;i++) //最左边到中线偏右位置4个
            {
                if(img_ptr[i] > THRESHOLD && img_ptr[i+1] < THRESHOLD && img_ptr[i+2] < THRESHOLD && img_ptr[i+3] < THRESHOLD)
                {
                    garage_mark++;
                }
            }
            if(garage_mark > 4)
            {
                garage_get_flag = 1;
                garage_circle_differ = 1;
                break;
            }
        }

        if( left_lose_raw > 25 && leftBorder[left_lose_raw+1] > 20 && !garage_circle_differ)//图像下部突然丢线  且非车库   环岛标志位1
        {
            circle_flag1 = 1;
            /*环岛降速*/
//            set_speed = 300;
//            kp = 58;
            //rt_mb_send(buzzer_mailbox, 500);//蜂鸣器发出声音！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        }
    }

    /*双边丢线 中线前5行为黑 判断三岔路*/
    else if(fork_road_flag == 1 && left_lose_flag > 0 && right_lose_flag > 0)//双边丢线且中线前5行变黑，认为三岔路
    {
        if(p == 0 || p == 1)
        {
            for(temp_h = 35; temp_h > 0; temp_h--)  //选择向左走
            {
                img_ptr = img[temp_h];
                road_mid = 10;
                img_ptr[road_mid] = 0;
                midLine[temp_h] = road_mid; //中线
            }
        }
        else{
            for(temp_h = 35; temp_h > 0; temp_h--)  //选择向右走
            {
                img_ptr = img[temp_h];
                road_mid = 180;
                img_ptr[road_mid] = 0;
                midLine[temp_h] = road_mid; //中线
            }
        }
     //   rt_event_send(fork_event, EVENT_FLAG5);
        fork_flag = 2;
        if(ex_fork_flag == 1 && fork_flag == 2)
        {
            ex_fork_flag = 0;
            fork_flag = 0;
            rt_sem_release(fork_sem);
            //rt_mb_send(buzzer_mailbox, 400);//蜂鸣器发出声音,提醒三岔识别位置
        }
    }
}

//根据中线控制偏差速度turn_speed输出,暂时使用比例控制，系数
short int motor_turn(uint8 mid[MT9V03X_H])
{
    int16 turn_speed = 0;//函数返回值
    uint8 i = 0;


    if(circle_flag1 && !circle_flag2) //第一次环岛标志直行
    {
        turn_speed = 0;
        return turn_speed;
    }
    if(circle_flag2 == 1)    //1或3时，非2
    {
        turn_speed = -185;    //出入环岛开环大左转，直到陀螺仪积分使退出
        return turn_speed;
    }

    if(circle_flag2 == 3)    //1或3时，非2
    {
        turn_speed = -65;    //出入环岛开环大左转，直到陀螺仪积分使退出
        return turn_speed;
    }

    if(circle_flag2 == 5)    //出环岛时看到标志位2,则只计算左边不丢线的图像后半部分！！！！！！！！！！！！！！！！！！！！！！！！！！！
    {
        /*
        for(i = 2; i < circle2_get_raw; i++)
        {
            turn_speed += mid[i];
        }
        turn_speed -= 84*(circle2_get_raw-2);
        return turn_speed;
        */
        //stop_flag = 2;
        turn_speed = 75;                          //重要 路肩出环岛防撞
        return turn_speed;
    }

    if(garage_get_flag == 1) //入库操作
    {
        garage_get_flag = 0;
        if(p == 0)  //已经经过了2次三岔,前提是开环出库打开摄像头，须放在看不到起止线之后
        {
            turn_speed = -23;   //目前第二圈看到起止线之后不再直行，直接转弯入库
            stop_flag = 1;
            return turn_speed;
        }
       else
        {
            turn_speed = 0;
            return turn_speed;
        }
    }

    for(i = 25;i < 40;i++)
    {
        turn_speed += mid[i];
    }
    turn_speed -= 1410;// (MT9V03X_W/2)*计算的行数，目前为35行，修改使用舵机行数时同时修改此参数
    turn_speed = (int16)(CAMERA_KP*turn_speed); //比例计算
    if(turn_speed >120)//限幅处理，防止转速过快
    {
        turn_speed = 120;
    }
    else if (turn_speed < -120) {
        turn_speed = -120;
    }

    return turn_speed;
}


