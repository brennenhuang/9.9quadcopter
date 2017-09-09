#include "lab.h"
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <FlashMemory.h>  //6.22
LSM9DS1 imu;

#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

#define PRINT_CALCULATED
//#define PRINT_RAW
//#define original
#define matchXsens 

#define PRINT_SPEED 250 // 250 ms between prints
#define DECLINATION 10.24   //預設:-8.58
#define RAD   57.296f

//在 main 裡面有用 timer 做切換，每 20ms 切換一次
extern bool flag;


// variable declaration


float calibration_data[9];   //各軸感測器的數值-offset;mpu6050就用calibration[6]
float calibration_offset[9]; //acc*3+gyro*3+mag*3,靜放並讀取1000次數據平均值
int count_for_calibration=0;
int countplus=0;  // 第二層迴圈的計數

//收集一段時間數值取平均拿來做校正
float sum_gx=0,sum_gy=0,sum_gz=0;  //int16_t sum_gx,sum_gy,sum_gz;
float sum_ax=0,sum_ay=0,sum_az=0;
float sum_mx=0,sum_my=0,sum_mz=0; 

float temp_gx,temp_gy,temp_gz;


float finally_roll=0.0;
float finally_pitch=0.0;
float finally_yaw=0.0;
float initial_roll=0.0;
float initial_pitch=0.0;
float initial_yaw=0.0;
float angle_x_acc=0.0;
float angle_y_acc=0.0;
float angle_x_gyro=0.0;
float angle_y_gyro=0.0;
float angle_z_gyro=0.0;
float PITCH,ROLL,YAW;

double Q[9];  //GYRO雜訊  實際要去測量這個白雜訊  C# L1.txt 12~20
double Ra[9]; //ACC雜訊 實際要去測量這個白雜訊  C# L1.txt 33~41
double Rm[9];  //MAG雜訊 實際要去測量這個白雜訊  C# L1.txt 54~62
double Cm[12];

//四元素
//NCF
static double q[]={1,0,0,0};

double P[]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};  //Identity 4X4 




double dT2=0.02;  //陀螺儀積分時間

double initial_RM[]={1,0,0,0,1,0,0,0,1};  //應該沒用到
double q_RM[9];

double R[9];
double M[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
double sensor_cali[63];  //應該用不到,因為是全身的程式一個方模有九個raw data,有七顆所以校正參數有63個


double q_ini[4];

//EKF:找出預估值(陀螺儀)跟量測值(加速度計)的自變異數(標準差平方),與共變異數
double Omega[16];
double Jaco_A[16];
double Jaco_A_t[16];
double Jaco_B[12];
double Jaco_B_t[12];
double halfomega[16];
double theta[16];
double omega_d,q_rms;
double I4[]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
double qa[4];
double temp_A[16],temp_A2[16];
double temp_B[12],temp_B2[12];
double temp_C[9],temp_C2[9];
double temp_D[4],temp_D2[4];
double temp_E[3],temp_F[3];
double temp_mx,temp_my,temp_mz;
double DCM_A[9],DCM_A_t[9];
double acc_d[3],mag_d[3];
double Z_error[3];

int Ea=10;
double Em=0.001;

double q_tes[4];
double eu_t0[3], eur_t0[3];
double initial_RM_t0[9];

double acc_cali;
double R_offset,P_offset,Y_offset;

/*   moving average:取五次EKF來做平均 */
double M_A_RateR[5],M_A_AttitudeY[5];
double M_A_RateP[5],M_A_AttitudeP[5];
double M_A_RateY[5],M_A_AttitudeR[5];
int moving_count=0;

bool imuInit()
{
   //雜訊的參數
  Q[0]=1.56493980000000e-06;
    Q[1]=0;
    Q[2]=0;
    Q[3]=0;
    Q[4]=1.2812551e-06;
    Q[5]=0;
    Q[6]=0;
    Q[7]=0;
    Q[8]=1.46095950000000e-06;

    Ra[0]=4.77107180000000e-07;
    Ra[1]=0;
    Ra[2]=0;
    Ra[3]=0;
    Ra[4]=2.0976752e-07;
    Ra[5]=0;
    Ra[6]=0;
    Ra[7]=0;
    Ra[8]=6.52815530000000e-07;

    Rm[0]=3.2890209e-05;
    Rm[1]=0;
    Rm[2]=0;
    Rm[3]=0;
    Rm[4]=4.15574570000000e-05;
    Rm[5]=0;
    Rm[6]=0;
    Rm[7]=0;
    Rm[8]=0.00010077412;

       //校正矩陣(磁力計)
    Cm[0]=3.59763662036754;
    Cm[1]=0.247017486504421;
    Cm[2]=0.0189925464210340;
    Cm[3]=(-0.962831259267916);
    Cm[4]=0;
    Cm[5]=3.50227377402372;
    Cm[6]=(-0.0273783959532506);
    Cm[7]=(-0.123135053744856);
    Cm[8]=0;
    Cm[9]=0;
    Cm[10]=3.60621804457917;
    Cm[11]=(-0.377940614692717);
    
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  imu.settings.gyro.sampleRate = 4;

  imu.settings.gyro.bandwidth = 0;

  imu.settings.accel.sampleRate = 4;
  imu.settings.accel.bandwidth = -1;   //6.22 1 or -1
  imu.settings.accel.highResEnable = true;
  imu.settings.accel.highResBandwidth = 0;
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1)
      ;
  }
  offset_calibration();
  for(int counter=0; counter<30; counter++)
  {
      EKF();
  }
  Initial_RM_t0(&R_offset,&P_offset,&Y_offset);
  return true;
}






unsigned char get_YawPitchRollInfo(float *yprAttitude, float *yprRate, float *xyzAcc,
                                   float *xyzGravity, float *xyzMagnet,int *a,unsigned long *time1,unsigned long *time2)   //6/22
{
//  char cmd[32];
//  if(Serial.available()>0)
//  {
//    
//    Serial.readBytes(cmd,32);
//    dT2=atof(cmd);
//    Serial.println(dT2);
//    delay(5000);
//  }
  double Y_ans, P_ans, R_ans;

// 
//  Serial.println();

//  delay(1);
if(flag == true)
{  
  *a=*a+1;
//  printf("aaaaaa=%d\n",*a);
  if(*a%3==1)
  {
      *time1=millis();
  }
  else if(*a%3==2)
  {
      *time2=millis();
  }
  else if(*a%3==0)
  {
    unsigned long interval=*time2-*time1;
    Serial.println(interval);
  }

 # if 1  //前後擺放問題(參考MPU9250擺後頭)
  EKF();
  Initial_RM_t0(&R_ans,&P_ans,&Y_ans);
   
  yprAttitude[0] = -(Y_ans);   //revise yaw無須offset
  yprAttitude[1] = (R_ans);
  yprAttitude[2] = P_ans;
 #endif
 #ifdef matchXsens 
  yprRate[0] = (temp_gy-calibration_offset[4]*180/PI);    //calibration_offset是脛度，關鍵是yprRate到底是脛度還是角度 目前把它視為脛度 ，此外temp_gx在EKF裡面有定義
  yprRate[1] = (temp_gx-calibration_offset[3]*180/PI);
  yprRate[2] = (temp_gz-calibration_offset[5]*180/PI);

  Serial.print(" yprRate[0]");
  Serial.print(yprRate[0],4);
  Serial.print(" yprRate[1]");
  Serial.print(yprRate[1],4);
  Serial.print(" yprRate[2]");
  Serial.println(yprRate[2],4);
  
#elif defined original
 
  yprRate[0] = -(imu.calcGyro(imu.gy)-calibration_offset[4]);
  yprRate[1] = -(imu.calcGyro(imu.gx)-calibration_offset[3]);
  yprRate[2] = imu.calcGyro(imu.gz)-calibration_offset[5];

  

#endif
 // EKF();
 // Initial_RM_t0(&R_ans,&P_ans,&Y_ans);
  // revise:8.1

//    Serial.print("yprAttitude[0]");
//    Serial.print(yprAttitude[0],4);
//    Serial.print("yprAttitude[1]");
//    Serial.print(yprAttitude[1],4);
//    Serial.print("yprAttitude[2]");
//    Serial.println(yprAttitude[2],4);

  
//  Serial.print(" Y_offset");
//  Serial.print(Y_offset,2);
//  Serial.print(" R_offset");
//  Serial.print(R_offset,2);
//  Serial.print(" P_offset");
//  Serial.println(P_offset,2);
    
}
  flag=false;
  return 0;
}


//算出加速度計跟陀螺儀 offset(總共跑2000個靜態的數值去平均)
void offset_calibration()
{
    for(int i = 0; i < 2000; i++)
    {
        imu.readGyro();
        imu.readAccel();
        imu.readMag();
        //將每秒轉幾度改成弳度
        sum_gx+=(imu.calcGyro(imu.gx)/180*PI);  
        sum_gy+=(imu.calcGyro(imu.gy)/180*PI);
        sum_gz+=(imu.calcGyro(imu.gz)/180*PI);
       
        sum_ax+=imu.calcAccel(imu.ax);  
        sum_ay+=imu.calcAccel(imu.ay);
        sum_az+=imu.calcAccel(imu.az);

        sum_mx+=imu.calcMag(imu.mx);  
        sum_my+=imu.calcMag(imu.my);
        sum_mz+=imu.calcMag(imu.mz);
        printf("%d\n",i);

       
    }
    calibration_offset[0]=((float)sum_ax)/2000;   //calibration[0]是ax的校正項
    calibration_offset[1]=((float)sum_ay)/2000;   //calibration[1]是ay的校正項
    calibration_offset[2]=((float)sum_az)/2000;   //calibration[2]是az的校正項
    calibration_offset[3]=((float)sum_gx)/2000;   //calibration[3]是gx的校正項
    calibration_offset[4]=((float)sum_gy)/2000;   //calibration[4]是gy的校正項
    calibration_offset[5]=((float)sum_gz)/2000;   //calibration[5]是gz的校正項  
    calibration_offset[6]=((float)sum_mx)/2000;   //calibration[6]是mx的校正項
    calibration_offset[7]=((float)sum_my)/2000;   //calibration[7]是my的校正項
    calibration_offset[8]=((float)sum_mz)/2000;   //calibration[8]是mz的校正項

    for(int i=0;i<9;i++)
    {
      printf("%f\t",calibration_offset[i]);    
    }
    printf("\n");
    acc_cali= sqrt(pow(calibration_offset[0],2)+pow( calibration_offset[1],2)+pow( calibration_offset[2],2));
}


void EKF()
{
#ifdef matchXsens
    // read acc data
    imu.readAccel();// 7.26 revise
    calibration_data[0]=(-imu.calcAccel(imu.ax))/acc_cali;   //  7.26 revise
    calibration_data[1]=imu.calcAccel(imu.ay)/acc_cali;
    calibration_data[2]=imu.calcAccel(imu.az)/acc_cali;
    
    // read gyro data
     imu.readGyro();

    temp_gx=imu.calcGyro(imu.gx);
    temp_gy=imu.calcGyro(imu.gy);
    temp_gz=imu.calcGyro(imu.gz);
    calibration_data[3]=-(temp_gx/180*PI-calibration_offset[3]);  //  7.26 revise
    calibration_data[4]=(temp_gy/180*PI-calibration_offset[4]);  //  7.26 revise
    calibration_data[5]=temp_gz/180*PI-calibration_offset[5];
    
    // read mag data 7/13
    imu.readMag();
    temp_mx=imu.calcMag(imu.mx);
    temp_my=imu.calcMag(imu.my);
    temp_mz=imu.calcMag(imu.mz);
#elif defined original
        imu.readAccel();// 7.26 revise
    calibration_data[0]=-(imu.calcAccel(imu.ax))/acc_cali;   //  7.26 revise
    calibration_data[1]=imu.calcAccel(imu.ay)/acc_cali;
    calibration_data[2]=imu.calcAccel(imu.az)/acc_cali;
    
    // read gyro data
     imu.readGyro();
    calibration_data[3]=-(imu.calcGyro(imu.gx)/180*PI-calibration_offset[3]);  //  7.26 revise
    calibration_data[4]=(imu.calcGyro(imu.gy)/180*PI-calibration_offset[4]);  //  7.26 revise
    calibration_data[5]=imu.calcGyro(imu.gz)/180*PI-calibration_offset[5];
    
    // read mag data 7/13
    imu.readMag();
    temp_mx=imu.calcMag(imu.mx);
    temp_my=imu.calcMag(imu.my);
    temp_mz=imu.calcMag(imu.mz);
    
#endif    
    
    calibration_data[6]=(temp_mx*Cm[0]+temp_my*Cm[1]+temp_mz*Cm[2]+Cm[3]);    //revise 之前打成calibration_data[7].8.9
    calibration_data[7]=temp_mx*Cm[4]+temp_my*Cm[5]+temp_mz*Cm[6]+Cm[7];
    calibration_data[8]=temp_mx*Cm[8]+temp_my*Cm[9]+temp_mz*Cm[10]+Cm[11];
    
    double tempp = -calibration_data[6];
    calibration_data[6] = -calibration_data[7];
    calibration_data[7] = tempp;
    //每次感測器的值經過calibration後,產出calibration_data[0]~calibration_data[9],順序為  gz,gy,gz,ax,ay,az,mx,my,mz
    //Omega為一個A=-AT矩陣
    Omega[0]=0,Omega[1]=(-calibration_data[3]),Omega[2]=(-calibration_data[4]),Omega[3]=(-calibration_data[5]);
    Omega[4]=calibration_data[3],Omega[5]=0,Omega[6]=calibration_data[5],Omega[7]=(-calibration_data[4]);    //7.6 revise
    Omega[8]=calibration_data[4],Omega[9]=(-calibration_data[5]),Omega[10]=0,Omega[11]=calibration_data[3];
    Omega[12]=calibration_data[5],Omega[13]=calibration_data[4],Omega[14]=(-calibration_data[3]),Omega[15]=0;

    //Jaco_A, dT2在NCF內
    for(int an = 0; an < 16; an++)
    {
      Jaco_A[an]=I4[an]+0.5*dT2*Omega[an];   //    7/6 revise 
    }

    //Jaco_B, q在NCF內

    Jaco_B[0]=-dT2*q[1];Jaco_B[1]=-dT2*q[2];Jaco_B[2]=-dT2*q[3];    //   7/6 revise 
    Jaco_B[3]=dT2*q[0];Jaco_B[4]=-dT2*q[3];Jaco_B[5]=dT2*q[2];
    Jaco_B[6]=dT2*q[3];Jaco_B[7]=dT2*q[0];Jaco_B[8]=-dT2*q[1];
    Jaco_B[9]=-dT2*q[2];Jaco_B[10]=dT2*q[1];Jaco_B[11]=dT2*q[0];

    for(int jb = 0; jb < 12; jb++)
    {
      Jaco_B[jb]=Jaco_B[jb]*0.5;    
    }

    MatrixTran(Jaco_A_t,Jaco_A,4,4);
    MatrixTran(Jaco_B_t,Jaco_B,3,4);

    //qa=Jaco_A*q  (4X4*4X1=4X1)
    MatrixMul(qa,Jaco_A,q,4,4,1);

    //calaulate new q_rms
    q_rms=sqrt(qa[0]*qa[0]+qa[1]*qa[1]+qa[2]*qa[2]+qa[3]*qa[3]);

    //算出新的四元素
    for(int i=0;i<4;i++)
    {
      q[i]=qa[i]/q_rms;
    }

    // temp_A2=Jaco_A*P*Jaco_A_t(4X4*4X4*4X4=4X4)
    // temp_B2=Jaco_B*Q*Jaco_B_t (4X3*3X3*3X4=4X4)
    MatrixMul(temp_A,Jaco_A,P,4,4,4);
    MatrixMul(temp_A2,temp_A,Jaco_A_t,4,4,4);
    MatrixMul(temp_B,Jaco_B,Q,4,3,3);
    MatrixMul(temp_B2,temp_B,Jaco_B_t,4,3,4);   //注意:C# 不是 temp_B2 而是 temp_A

    //P = Jaco_A * P * Jaco_A_t + Jaco_B * Q * Jaco_B_t 
    for(int an=0;an<16;an++)
    {
      P[an]=temp_A2[an]+temp_B2[an];
    }

    //用四元素算出旋轉矩陣存入DCM_A
    Q_RM(DCM_A);

    MatrixTran(DCM_A_t,DCM_A,3,3);

    // 加入適應性調變誤差 
    //震幅誤差
    double AA_g[]={0,0,1};
    double RA_m=sqrt(pow(0,2)+pow(0,2)+pow(1,2));
    double A_m=sqrt(calibration_data[0]*calibration_data[0]+calibration_data[1]*calibration_data[1]+calibration_data[2]*calibration_data[2]);
    double E_m=max(A_m/RA_m,RA_m/A_m);
    //方向誤差
    double RA_d[3]={AA_g[0]/RA_m,AA_g[1]/RA_m,AA_g[2]/RA_m};
    
    //acc preprocessing

    double acc_rms=sqrt(calibration_data[0]*calibration_data[0]+calibration_data[1]*calibration_data[1]+calibration_data[2]*calibration_data[2]);

    //calibration_data normalize
    for(int an=0;an<3;an++)
    {
      acc_d[an]=calibration_data[an]/acc_rms;
    }
     MatrixMul(temp_E,DCM_A_t,RA_d,3,3,1);
     double E_d=sqrt(pow(temp_E[0]-acc_d[0],2)+pow(temp_E[1]-acc_d[1],2)+pow(temp_E[2]-acc_d[2],2));
     double EE_A=E_d*E_m;
     double EA_cov=exp(10*EE_A);  //5是可調整變數
     if(EA_cov>=1e+10)
     {
         EA_cov=1e+10;
     }
    //權重調變結束
     //EA_cov=1;

    
    // acceleration update 
    double H[]={-2*q[2],2*q[3],-2*q[0],2*q[1],2*q[1],2*q[0],2*q[3],2*q[2],2*q[0],-2*q[1],-2*q[2],2*q[3]};
    double H_t[12];
    MatrixTran(H_t,H,4,3);
    double Z_predict[]={2*(q[1]*q[3]-q[0]*q[2]),2*(q[2]*q[3]+q[0]*q[1]),q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]};
    double S[9];
    double S_inv[9];
    double KG[12];

    //Kalman gain 
    //S=H*P*H' + Ra/Ea*EA_cov
    MatrixMul(temp_B,P,H_t,4,4,3);
    MatrixMul(temp_C,H,temp_B,3,4,3);
    for(int an=0;an<9;an++)
    {
      S[an]=temp_C[an]+EA_cov*Ra[an]/Ea;    // 7/6 revise
    }

    //K = P*H'*inv(S)
    MatrixInv3(S_inv,S);
    MatrixMul(KG,temp_B,S_inv,4,3,3);

    //state updata acc
    //q=q+K*(z-H*q)
    for(int an=0;an<3;an++)
    { //dz = z-H*q
      Z_error[an]=acc_d[an]-Z_predict[an];
    }
    MatrixMul(temp_D,KG,Z_error,4,3,1);
    for(int an=0;an<4;an++)
    { //q_new = q+K*dz
      qa[an]=q[an]+temp_D[an];
    }
    //copy qa to q
    memcpy(q,qa,sizeof(qa));
    q_rms=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);

    for(int i=0;i<4;i++)
    {
      q[i]=q[i]/q_rms;
    }

    //P = (I-K*H)*P
    MatrixMul(temp_A,KG,H,4,3,4);
    for(int an=0;an<16;an++)
    {
      temp_A[an]=I4[an]-temp_A[an];      // 7/6 revise 
    }
    MatrixMul(temp_A2,temp_A,P,4,4,4);
    memcpy(P,temp_A2,sizeof(temp_A2));//P_new

    Q_RM(DCM_A);
    MatrixTran(DCM_A_t,DCM_A,3,3);

    double z[3];
    //mag preprocessing
    double zv[]={2*q[1]*q[3]-2*q[0]*q[2],2*q[2]*q[3]+2*q[0]*q[1],q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]};
    double mz=calibration_data[6]*zv[0]+calibration_data[7]*zv[1]+calibration_data[8]*zv[2];
    for(int an=0;an<3;an++)
    {
      z[an]=calibration_data[6+an]-mz*zv[an];
    }

    double mag_rms=sqrt(z[0]*z[0]+z[1]*z[1]+z[2]*z[2]);

    double MM_G[3]={0,mag_rms,0};
    //適應性權重調變(磁力計)
    double M_m=mag_rms;
    double RM_m=mag_rms;
    E_m=max(M_m/RM_m,RM_m/M_m);
    
    for(int an=0;an<3;an++)
    {
      //Rmag_d[an]=sum_iner[an+6]/Rmag_rms;//
      mag_d[an]=z[an]/mag_rms;
    }

     
    double RM_d[3]={MM_G[0]/RM_m,MM_G[1]/RM_m,MM_G[2]/RM_m};
    MatrixMul(temp_F,DCM_A_t,RM_d,3,3,1);
    E_d=sqrt(pow(temp_F[0]-mag_d[0],2)+pow(temp_F[1]-mag_d[1],2)+pow(temp_F[2]-mag_d[2],2));
    double EE_M=E_d*E_m;
    double EM_cov=exp(20*EE_M);
    if(EM_cov>=1e+7)
    {
        EM_cov=1e+7;
    }
   // EM_cov=1;
//MAG UPDATE
      
    double H_M[]={2*q[3],2*q[2],2*q[1],2*q[0],2*q[0],-2*q[1],2*q[2],-2*q[3],-2*q[1],-2*q[0],2*q[3],2*q[2]};
MatrixTran(H_t,H_M,4,3);
    double Z_M_predict[]={2*(q[1]*q[2]+q[0]*q[3]),q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3],2*(q[2]*q[3]-q[0]*q[1])};

//Kalman gain mag
//S=H*P*H' + Rm/Em*EM_cov
    MatrixMul(temp_B,P,H_t,4,4,3);
    MatrixMul(temp_C,H_M,temp_B,3,4,3);
    for(int an=0;an<9;an++)
    {
      S[an]=temp_C[an]+EM_cov*Rm[an]/Em;
    }

//K = P*H'*inv(S)
    MatrixInv3(S_inv,S);
    MatrixMul(KG,temp_B,S_inv,4,3,3);
//state update mag
    for(int an=0;an<3;an++)
    {
      Z_error[an]=mag_d[an]-Z_M_predict[an];
    }
    MatrixMul(temp_D,KG,Z_error,4,3,1);
    double filter[16] ={1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};   //讓磁力季更新不影響q1,q2  7.27 revise
    MatrixMul(temp_D2,filter,temp_D,4,4,1);
    for(int an=0;an<4;an++)
    {
      qa[an]=q[an]+temp_D2[an];
    }
    memcpy(q,qa,sizeof(qa));
    q_rms=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    for(int i=0;i<4;i++)
    {
      q[i]=q[i]/q_rms;
    }
    MatrixMul(temp_A,KG,H_M,4,3,4);
    for(int an=0;an<16;an++)
    {
     temp_A[an]=I4[an]-temp_A[an];
    }     
    MatrixMul(temp_A2,temp_A,P,4,4,4);
    memcpy(P,temp_A2,sizeof(temp_A2));   
}

//matrix operation

int MatrixTran(double out_AT[], double in_A[], int out_row, int out_column)
{
  for (int i = 0; i < out_row; i++)
    for (int j = 0; j < out_column; j++)
      out_AT[i*out_column+j] = in_A[j*out_row+i];
  return 0;
}

int MatrixMul(double out_multi[], double L[], double R[], int m, int n, int p)    //m*n X n*p
{
  for (int i = 0; i < m; i++)
  {
    for (int j = 0; j < p; j++)
    {
        out_multi[i*p+j] = 0;
      for (int k = 0; k < n; k++)
      {
        out_multi[i*p+j] += L[i*n+k] * R[k*p+j];
      }
    }
  }
  return 0;
}

int MatrixInv3(double AI[], double A[])
{
  double det=A[0]*A[4]*A[8]+A[1]*A[5]*A[6]+A[2]*A[3]*A[7]-A[2]*A[4]*A[6]-A[1]*A[3]*A[8]-A[0]*A[5]*A[7];
  if(det==0)
  return 0;
  AI[0] = (A[4]*A[8]-A[5]*A[7])/det;  AI[1] = (A[2]*A[7]-A[1]*A[8])/det;  AI[2] = (A[1]*A[5]-A[2]*A[4])/det;  
  AI[3] = (A[5]*A[6]-A[3]*A[8])/det; AI[4] = (A[0]*A[8]-A[2]*A[6])/det;  AI[5] = (A[2]*A[3]-A[0]*A[5])/det;  
    AI[6] = (A[3]*A[7]-A[4]*A[6])/det;  AI[7] = (A[1]*A[6]-A[0]*A[7])/det;  AI[8] = (A[0]*A[4]-A[1]*A[3])/det;  
  return 0;
}

//四元素表達旋轉矩陣
int Q_RM(double q_RM[])
{
  q_RM[0]=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
  q_RM[1]=2*(q[1]*q[2]-q[0]*q[3]);
  q_RM[2]=2*(q[1]*q[3]+q[0]*q[2]);
  q_RM[3]=2*(q[1]*q[2]+q[0]*q[3]);
  q_RM[4]=q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3];
  q_RM[5]=2*(q[2]*q[3]-q[0]*q[1]);
  q_RM[6]=2*(q[1]*q[3]-q[0]*q[2]);
  q_RM[7]=2*(q[2]*q[3]+q[0]*q[1]);
  q_RM[8]=q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
  return 0;
}

void Initial_RM_t0(double *r,double *p,double *y){      //initial rotation matrix & euler angle
      q_tes[0]=q[0];q_tes[1]=q[1];q_tes[2]=q[2];q_tes[3]=q[3];
      initial_RM_t0[0]=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
      initial_RM_t0[1]=2*(q[1]*q[2]-q[0]*q[3]);
      initial_RM_t0[2]=2*(q[1]*q[3]+q[0]*q[2]);
      initial_RM_t0[3]=2*(q[1]*q[2]+q[0]*q[3]);
      initial_RM_t0[4]=q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3];
      initial_RM_t0[5]=2*(q[2]*q[3]-q[0]*q[1]);
      initial_RM_t0[6]=2*(q[1]*q[3]-q[0]*q[2]);
      initial_RM_t0[7]=2*(q[2]*q[3]+q[0]*q[1]);
      initial_RM_t0[8]=q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
/*
      Serial.print("  initial_RM_t0[0]: ");    Serial.print(initial_RM_t0[0]);
    Serial.print("  initial_RM_t0[1]: ");    Serial.print(initial_RM_t0[1]);
    Serial.print("  initial_RM_t0[2]: ");    Serial.print(initial_RM_t0[2]);
    Serial.print("  initial_RM_t0[3]: ");    Serial.print(initial_RM_t0[3]);
    Serial.print("  initial_RM_t0[4]: ");    Serial.print(initial_RM_t0[4]);
    Serial.print("  initial_RM_t0[5]: ");    Serial.print(initial_RM_t0[5]);
    Serial.print("  initial_RM_t0[6]: ");    Serial.print(initial_RM_t0[6]);
    Serial.print("  initial_RM_t0[7]: ");    Serial.print(initial_RM_t0[7]);
    Serial.print("  initial_RM_t0[8]: ");    Serial.print(initial_RM_t0[8]);
    */

      eu_t0[0]=atan2(initial_RM_t0[7],initial_RM_t0[8]);
      eu_t0[1]=asin(-1*initial_RM_t0[6]);
      eu_t0[2]=atan2(initial_RM_t0[3],initial_RM_t0[0]);

      eur_t0[0]=180*eu_t0[0]/PI;    //raw     x
      eur_t0[1]=180*eu_t0[1]/PI;      //pitch   y
      eur_t0[2]=180*eu_t0[2]/PI;    //yaw     z

    //  eur_t0[2] -= DECLINATION * PI / 180;
  /*                                                           //7.26 revise
      if (eur_t0[2] > PI) eur_t0[2] -= (2 * PI);
      else if (eur_t0[2] < -PI) eur_t0[2] += (2 * PI);
      else if (eur_t0[2] < 0) eur_t0[2] += 2 * PI;
  */
      // Convert everything from radians to degrees:
    //  eur_t0[2] *= 180.0 / PI;

      //eur_t0[2]=eur_t0[2]-180;

      *r=eur_t0[0];
      *p=eur_t0[1];
      *y=eur_t0[2];
 #if 1     
      Serial.print("R:");
      Serial.print(eur_t0[0],2);
      Serial.print("P:");
      Serial.print(eur_t0[1],2);
      Serial.print("Y:");
      Serial.println(eur_t0[2],2);
 #endif     
//      printf("R:%f\t",eur_t0[0]);
//      printf("P:%f\t",eur_t0[1]);
//      printf("Y:%f\n",eur_t0[2]);      
    }
