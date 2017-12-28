/** @copyright XunFang Communication Tech Limited. All rights reserved. 2013.
  * @file  filter.c
  * @author  hzp
  * @version  V1.0.0
  * @date  12/28/2017
  * @brief  ��ת������������
  */




/**
  * @brief  ��32λ������������ת��Ϊ������
  * @details  
        Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
        R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա�� 
  * @param  ResrcData  ��Ҫ���˵�����
  * @param  

  * @return ����֮�������
 */
 double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{

    double R = MeasureNoise_R;
    double Q = ProcessNiose_Q;

    static double x_last;
    double x_mid = x_last;
    double x_now;

    static double p_last;
    double p_mid ;
    double p_now;

    double kg;

    x_mid=x_last;                       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=p_last+Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����

    /*
     *  �������˲��������Ҫ��ʽ
     */
    kg=p_mid/(p_mid+R);                 //kgΪkalman filter��R Ϊ����
    x_now=x_mid+kg*(ResrcData-x_mid);   //���Ƴ�������ֵ
    p_now=(1-kg)*p_mid;                 //����ֵ��Ӧ��covariance
    p_last = p_now;                     //����covariance ֵ
    x_last = x_now;                     //����ϵͳ״ֵ̬

    return x_now;

}

/**
  * @brief ������Ȩ�˲��㷨
  * @details 
  * @gram p_buff �����������
  * @gram  value ����ֵ
  * @gram  n_sample �������� �趨�ò��ܸĶ�
  * @retval ��
 */

float huadongjiaquan(float *p_buff,float value, int n_sample)
{
  
    float temp;
    float sum=0;
     for(int i=1;i < n_sample; i++)
     {
        p_buff[i-1] = p_buff[i];
        sum += p_buff[i] * i;
     }
     p_buff[n_sample-1] = value;
     sum += value*n_sample;
     
      temp = sum /(((n_sample+1)/2 * n_sample)-1);
     return temp;
     
}