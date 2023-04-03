#include "camera_process.h"
#include "fork.h"
#include "buzzer.h"
#include "motor.h"
#include "headfile.h"

uint8 leftBorder[MT9V03X_H];
uint8 rightBorder[MT9V03X_H];
uint8 midLine[MT9V03X_H];

/*������־λ����������*/
uint8 circle_flag1 = 0;  //����ͻȻ���߱�־
uint8 circle_flag2 = 0;  //����������������־���������ǻ��ֺ��˳���־��
uint8 circle2_get_raw = 0;

uint8 garage_get_flag = 0; //Ѱ�������־
uint8 garage_circle_differ = 0; //����ͻ�������
uint8 stop_flag = 0; //ͣ����־

uint8 fork_flag = 0;  //�޶�����ͨ����Ϊ1������·Ϊ2
uint8 ex_fork_flag = 0;

//�߽紦��,Ѱ������
void getborder(uint8 (*img)[MT9V03X_W])
{
    uint8 temp_w = 0; //����
    uint8 temp_h = 0;
    uint8* img_ptr = img[0];
    uint8 road_mid = 94; //ֵΪMT9V03X_W / 2; �״�Ѱ���ߴ��м俪ʼ,֮������������,�ɾݴ����������ж�
    uint8 road_mid_new; //����һ�����߱Ƚϱ�������ͻ��

    uint8 circle_flag_temp = 0;  //������ʱ�жϱ�־

    uint8 left_lose_flag = 0;  //��߶�����1������Ѱ������2
    uint8 right_lose_flag = 0; //�ұ߶�����1������Ѱ������2
    uint8 left_border_flag = 0;//����ҵ��߽��־
    uint8 right_border_flag = 0;//�ұ��ҵ��߽��־
    uint8 fork_road_flag = 0; //����·��־
    uint8 fork_road_raw = 0;  //����·�ֲ���
    uint8 left_lose_raw = 0;   //��߿�ʼ������
    uint8 left_get_raw = 0;    //��������ҵ��ߵ���
    uint8 right_lose_raw = 0;  //�ұ߿�ʼ������
    uint8 right_get_raw = 0;    //��������ҵ��ߵ���

    uint8 garage_mark = 0; //����ڰ������־

    uint8 i = 0;//test

    for (temp_h = 49 ; temp_h > 0; temp_h--)  //Ѱ��50��ͼ������,forѭ���ڲ����Ƿ���
    {
        if(temp_h > 25) //�ж�����·
        {
            img_ptr = img[temp_h - 5];
            if( road_mid > 70 &&  img_ptr[road_mid] < THRESHOLD && img_ptr[road_mid-1] < THRESHOLD && !circle_flag1 && !circle_flag2)//��һ�е�������ǰ��5�г��������߽磡����Ϊ����·,����ȥ�����У�������
            {
                 fork_road_flag = 1;
                 fork_road_raw  = temp_h;
            }
        }
        img_ptr = img[temp_h];
        left_border_flag = 0;
        right_border_flag = 0;

        //��ʼ����
        for (temp_w = road_mid; temp_w < MT9V03X_W; temp_w++)
        {
            if (img_ptr[temp_w] < THRESHOLD && img_ptr[temp_w - 1] < THRESHOLD)
            {
                rightBorder[temp_h] = temp_w - 2;
                img_ptr[temp_w - 2] = 0; //��ʾ�ұ߽�
                img_ptr[temp_w - 3] = 0; //��ʾ�ұ߽�
                img_ptr[temp_w - 4] = 0; //��ʾ�ұ߽�
                right_border_flag = 1;
                break;
            }
        }
        if(right_border_flag == 0)//û�ѵ��ұ߽�
        {
            rightBorder[temp_h] = 186;
            img_ptr[186] = 0; //��ʾ�ұ߽�
            img_ptr[185] = 0; //��ʾ�ұ߽�
            img_ptr[184] = 0; //��ʾ�ұ߽�

            if(right_lose_flag == 0)//�״�û�ѵ��ұ߽�
            {
                right_lose_flag = 1;     //�ұ߽綪ʧ
                right_lose_raw = temp_h; //��¼�״ζ��ߵ���
            }
        }

        if(right_border_flag == 1 && right_lose_flag == 1)  //���ߺ�����Ѱ��
        {
            right_lose_flag = 2;  //�߽綪ʧ���һ�
            right_get_raw = temp_h; //�����һ���λ��
        }

        for (temp_w = road_mid; temp_w > 0; temp_w--)
        {
            if (img_ptr[temp_w] < THRESHOLD && img_ptr[temp_w + 1] < THRESHOLD)
            {
                leftBorder[temp_h] = temp_w + 2;
                img_ptr[temp_w + 2] = 0; //��ʾ��߽�
                img_ptr[temp_w + 3] = 0; //��ʾ��߽�
                img_ptr[temp_w + 4] = 0; //��ʾ��߽�
                left_border_flag = 1;
                break;
            }
        }
        if(left_border_flag == 0)//û�ѵ���߽�
        {
            leftBorder[temp_h] = 2;
            img_ptr[2] = 0; //��ʾ��߽�
            img_ptr[3] = 0; //��ʾ��߽�
            img_ptr[4] = 0; //��ʾ��߽�

            if(left_lose_flag == 0)//û�ѵ���߽�ĵ�һ��
            {
                left_lose_flag = 1;     //��߽綪ʧ
                left_lose_raw = temp_h; //����λ��
            }
        }

        if(left_border_flag == 1 && left_lose_flag == 1)  //���ߺ�����Ѱ��
        {
            left_lose_flag = 2;  //�߽綪ʧ���һ�
            left_get_raw = temp_h; //�����һ���λ��
        }

    /*
     *  if(rightBorder[temp_h] - leftBorder[temp_h] < 2)//���������Ӳ෽�ճ�����ǰ��һƬ�ڣ����������Ӧ����ȫ�ڵı߽�
        {
            road_mid = 94;
        }
    */

        else{
        road_mid_new = (leftBorder[temp_h] + rightBorder[temp_h]);//������ߣ������߽�Ϊͼ��߽磬�ں������
        road_mid_new >>= 1;   //ִ�г��Զ�

        if(road_mid_new - road_mid >15) road_mid = road_mid_new - 15;//�����޷�
        else if(road_mid_new - road_mid < -15) road_mid = road_mid_new + 15;
        else road_mid = road_mid_new;
        }

        if(img_ptr[road_mid] == 0)//��ָͻȻ�����ܵ�ͼ������߻�ͦ�ȶ�����������������ܵ���������
        {
            road_mid = midLine[temp_h - 2];//���߻�ȥ������
        }
        midLine[temp_h] = road_mid; //����
        img_ptr[road_mid] = 0;  //������õ������߱�Ϊ��ɫ
    }

    /*��������ʱ�������*/


    /*������־2����*/
    if ( circle_flag1 == 1 && left_lose_flag == 2 )//�ڱ�־1�� ��߿�ʼ���ߣ�����Ѱ��,ȥ���ұ߲����ߵ��ж�
    {
        if( leftBorder[left_get_raw] > 30) //����ͻȻѰ��
         {
            switch(circle_flag2){
            case 0:
                circle_flag2 = 1;         //ʶ�𵽻���������������������������
                //rt_mb_send(buzzer_mailbox, 300);//������������������������������������������������������������������������������������������������
                break;
            case 4:
                circle_flag2 = 5;         //��ȫֱ�г�����
                //rt_mb_send(buzzer_mailbox, 300);//������������������������������������������������������������������������������������������������
                break;
            default:
                break;
            }
            //circle_flag1 = 0;  //������־λ�ڳ�����ǰһֱ����0
         }
    }

    if(circle_flag2 == 5)
    {
        circle2_get_raw = left_get_raw; //��ʱ������������־λ2ֻ�����ҵ�����ߺ����Ҷ��е�һ����
    }

    ex_fork_flag = fork_flag;

    /*�޶��ߣ���Ϊ��ͨ����*/
    if(left_lose_flag == 0 && right_lose_flag == 0)
    {
        fork_flag = 1;
        garage_circle_differ = 0;//��������ͻ����Ļ���
        //rt_event_send(fork_event, EVENT_FLAG3);

        if(circle_flag2 == 5)  //����֮����ͨ�����˳�����
        {
            circle_flag1 = 0;
            circle_flag2 = 0;
            /*����������*/
//            set_speed = 350;
//            kp = 65;
        }
    }

    /*ֻ�� ��߶��� �жϳ��� �� ����(�ų�ʮ��Ӱ��)*/
    else if(left_lose_flag > 0 && right_lose_flag == 0)
    {
        for(temp_h = left_lose_raw; temp_h > 0; temp_h--)
        {
            garage_mark = 0;
            img_ptr = img[temp_h];
            for(i=0;i<135;i++) //����ߵ�����ƫ��λ��4��
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

        if( left_lose_raw > 25 && leftBorder[left_lose_raw+1] > 20 && !garage_circle_differ)//ͼ���²�ͻȻ����  �ҷǳ���   ������־λ1
        {
            circle_flag1 = 1;
            /*��������*/
//            set_speed = 300;
//            kp = 58;
            //rt_mb_send(buzzer_mailbox, 500);//������������������������������������������������������������������������������������������������
        }
    }

    /*˫�߶��� ����ǰ5��Ϊ�� �ж�����·*/
    else if(fork_road_flag == 1 && left_lose_flag > 0 && right_lose_flag > 0)//˫�߶���������ǰ5�б�ڣ���Ϊ����·
    {
        if(p == 0 || p == 1)
        {
            for(temp_h = 35; temp_h > 0; temp_h--)  //ѡ��������
            {
                img_ptr = img[temp_h];
                road_mid = 10;
                img_ptr[road_mid] = 0;
                midLine[temp_h] = road_mid; //����
            }
        }
        else{
            for(temp_h = 35; temp_h > 0; temp_h--)  //ѡ��������
            {
                img_ptr = img[temp_h];
                road_mid = 180;
                img_ptr[road_mid] = 0;
                midLine[temp_h] = road_mid; //����
            }
        }
     //   rt_event_send(fork_event, EVENT_FLAG5);
        fork_flag = 2;
        if(ex_fork_flag == 1 && fork_flag == 2)
        {
            ex_fork_flag = 0;
            fork_flag = 0;
            rt_sem_release(fork_sem);
            //rt_mb_send(buzzer_mailbox, 400);//��������������,��������ʶ��λ��
        }
    }
}

//�������߿���ƫ���ٶ�turn_speed���,��ʱʹ�ñ������ƣ�ϵ��
short int motor_turn(uint8 mid[MT9V03X_H])
{
    int16 turn_speed = 0;//��������ֵ
    uint8 i = 0;


    if(circle_flag1 && !circle_flag2) //��һ�λ�����־ֱ��
    {
        turn_speed = 0;
        return turn_speed;
    }
    if(circle_flag2 == 1)    //1��3ʱ����2
    {
        turn_speed = -185;    //���뻷����������ת��ֱ�������ǻ���ʹ�˳�
        return turn_speed;
    }

    if(circle_flag2 == 3)    //1��3ʱ����2
    {
        turn_speed = -65;    //���뻷����������ת��ֱ�������ǻ���ʹ�˳�
        return turn_speed;
    }

    if(circle_flag2 == 5)    //������ʱ������־λ2,��ֻ������߲����ߵ�ͼ���벿�֣�����������������������������������������������������
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
        turn_speed = 75;                          //��Ҫ ·���������ײ
        return turn_speed;
    }

    if(garage_get_flag == 1) //������
    {
        garage_get_flag = 0;
        if(p == 0)  //�Ѿ�������2������,ǰ���ǿ������������ͷ������ڿ�������ֹ��֮��
        {
            turn_speed = -23;   //Ŀǰ�ڶ�Ȧ������ֹ��֮����ֱ�У�ֱ��ת�����
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
    turn_speed -= 1410;// (MT9V03X_W/2)*�����������ĿǰΪ35�У��޸�ʹ�ö������ʱͬʱ�޸Ĵ˲���
    turn_speed = (int16)(CAMERA_KP*turn_speed); //��������
    if(turn_speed >120)//�޷�������ֹת�ٹ���
    {
        turn_speed = 120;
    }
    else if (turn_speed < -120) {
        turn_speed = -120;
    }

    return turn_speed;
}


