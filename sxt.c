 #include "ftm.h"
#include "common.h"
#define H 320                            //采集的图像的列数
#define V 51                              //采集的图像的行数
#define Hx 70
#define N 3
int fabss(int n);
#define MAX(a,b)            (((a) > (b)) ? (a) : (b))
#define Min(a,b)            (((a) < (b)) ? (a) : (b))
extern unsigned char IsStartLine ;
unsigned char BlackRow=0;
unsigned char WhiteRow=0;
unsigned char WhiteRow1=0;
unsigned char BlackEndL = 0;
unsigned char BlackEndM = 0;
unsigned char BlackEndR = 0;
unsigned char BlackEndMax = 0;
unsigned char BlackEndLMR= 0;
int StableNumbers = 0;
int StableNumbers2 = 0;
int CompensateCount = 0;

int LeftStableNumbers = 0;
int RightStableNumbers = 0;
int ValidLineCount = 0;
int ValidLineCount1 = 0; //左边缘找到并且右边找到
int ValidLineCount2 = 0;//左边缘找到或者右边找到

int Excursion;
int ValidExcursionCount=0;
unsigned char LineType[V];
		
double MidLineVariance = 0.0;
int MidLineExcursion;
signed char TripPointPos[V];
signed char SubValue[V];

extern s16 CurrentVelocity;
extern unsigned char Pix_Data[V][H]; 
extern unsigned char Pix_Data1[V][Hx];
extern unsigned char Pix_Data2[V][Hx];
int LeftBlack[V]; //左黑线							   
 int RightBlack[V];//右黑线
 int BlackLineData[V];//中心线

extern int b;
float temerror=0.0;
int countblack1=0;
float value_buf[N];
unsigned char ThresholdData[V][Hx];  //?t?μ?ˉêy?Yêy×é		
/////////////////////////////////////////////////////////////////////////////////////
int g_Derict=0; 
int g_BasePos = Hx/2;
unsigned char ValidLineR[V]={0}; //ê?×?μàóò±?DDóDD§±ê??êy×é
unsigned char ValidLineL[V]={0};//ê?×?μà×ó±?DDóDD§±ê??êy×é
unsigned char NoValidLMax = 0;//ê?×???2?μà×ó±?á?D??a????êy
unsigned char NoValidRMax = 0;//ê?×???2?μàóò±?á?D??a????êy
unsigned char CrossingStable = 0;
unsigned char IsCrossing=0;

int LCrossingTripPos = 0;
int RCrossingTripPos = 0;
//?òμ?ê?×?μ?ì?±?μ?
unsigned char bFoundTripPoint = 0;
int startPos=0,endPos=0;
//////////////////////////////////////////////////////////////////////////////////////
signed char RoadType = -1;
signed char LastRoadType = 0;
int ElementCount2 = 0;
//í·?2????
int Head2 = 0,Rear2 = 0;
#define Size2 30
//?óáDêy×é
signed char RoadTypeData2[Size2] = {0};
int AllStraightCount = 0;
int AllSmallSCount = 0;
int AllBigSCount = 0;
int AllBendCount = 0;
unsigned char StandardRoadType = 1;
int Foresight = 15;
int HighSpeedFlag = 0;
int StopNowCount=0;
int HighSpeedCount = 0;
int StraightSpeed = -75;
int	SmallSSpeed	 = -70;
int	BigSSpeed  =-70;
int	BendSpeed = -70;   // í?μà?ù?è
int	CommonSpeed = -70;
int	StraightToBendSpeed = -70;

//int SpecialSpeed = -15;

int IncreaseSpeed1 = 0;//3¤?±μà?ù?èìáéy1
int IncreaseSpeed2 = 0;//3¤?±μà?ù?èìáéy2
int IncreaseSpeed3 = 0;//3¤?±μà?ù?èìáéy3
int CanSpeedUp = 0;	 //3¤?±μàìá?ùê1?ü
int CanFullSpeed = 0;//?éè??ùê1?ü
unsigned char StraightFS=19;//?±μà?°?°//10
unsigned char SmallSFS=19;//D?s?°?°
unsigned char BigFS=19;//′óS?°?°6
unsigned char BendFS=19;//?±í??°?°
unsigned char CommonFS=19;//??í¨?°?°
unsigned char StraightToBendFS=19;

unsigned char StraightEnd=V-12;
unsigned char SmallSEnd=V-12;
unsigned char BigSEnd=V-12;
unsigned char BendEnd=V-12;
unsigned char CommonEnd=V-12;
unsigned char StraightToBenEnd = V-12;
int StraightToBendCount = 0;
s16 SetPoint=0,speed=0,SetPoint1=0;
//float Kp=8;
//////////////////////////////////////////////////////////////////////////////////////
int otsu (unsigned char *image, int rows, int cols, int x0, int y0, int dx, int dy, int vvv)//用加权平均值求阈值
{       
	unsigned char *np; // 图像指针
	int thresholdValue=1; // 阈值
	int ihist[256] = {0}; // 图像直方图，256个点

	int i, j, k; // various counters
	int n, n1, n2, gmin, gmax;
	double m1, m2, sum, csum, fmax, sb;

	//gmin=255; gmax=0;//图像阈值范围设置初始
	gmin=200; gmax=60;
	// 生成直方图
	for (i = y0 + 1; i < y0 + dy - 1; i++) {
	   np = &image[i*cols+x0+1];
	   for (j = x0 + 1; j < x0 + dx - 1; j++) {
		   ihist[*np]++;
		   if(*np > gmax) gmax=*np;
		   if(*np < gmin) gmin=*np;
		   np++; /* next pixel */
	   }
	}

	// set up everything
	sum = csum = 0.0;
	n = 0;

	for (k = 0; k <= 255; k++) {
	   sum += (double) k * (double) ihist[k]; /* x*f(x) 质量矩*/
	   n += ihist[k]; /* f(x) 质量 */
	}

	if (!n) {
	   // if n has no value, there is problems...
	   //fprintf (stderr, "NOT NORMAL thresholdValue = 160\n");
	   return (160);
	}

	// do the otsu global thresholding method
	fmax = -1.0;
	n1 = 0;
	for (k = 0; k < 255; k++) 
        {         
	   n1 += ihist[k];
	   if (!n1)  { continue; }
	   n2 = n - n1;
	   if (n2 == 0)  { break; }
	   csum += (double) k *ihist[k];
	   m1 = csum / n1;
	   m2 = (sum - csum) / n2;
	   sb = (double) n1 *(double) n2 *(m1 - m2) * (m1 - m2);
	   /* bbg: note: can be optimized. */
	   if (sb > fmax)
           {
		   fmax = sb;
		   thresholdValue = k;
	   } 
	}

	return(thresholdValue);
} 


void fz()
{
  int i,j;
  for(i=0;i<51;i++)
    for(j=0;j<70;j++)
      Pix_Data2[i][j]=ThresholdData[i][j];
}






void HandleImg()
{
	unsigned char *p=0;
        unsigned char *q=0;
  for (p = &Pix_Data1[0][0],q = &ThresholdData[0][0]; p <= &Pix_Data1[V-1][Hx-1]; p++,q++)
	{
		if (*p >b){*q = 255;}
		else{*q = 0;}
	}
      fz();
}





//对噪点取均值滤波
       void lvbo (unsigned char ImageData[][70])
        { int i,j;
	for (i = 1;i < 50;i++)
	{
		for (j = 1;j < 69;j++)
		{
			if (ImageData[i][j] < 10 || ImageData[i][j] > 240)
			{
				ImageData[i][j] = (ImageData[i-1][j-1]+ImageData[i-1][j]+ImageData[i-1][j+1]+ImageData[i][j-1]+ImageData[i][j+1]+ImageData[i+1][j-1]+ImageData[i+1][j]+ImageData[i+1][j+1])/8;
			}
		}
	}
        }



     void hsz ()
         {   int i,j,j1;
           for (i = 0;i < 51;i++)//////////////////////
	     {
               
             
		for (j =20,j1=0;j <=296;j=j+4,j1++)
		{
                  Pix_Data1[i][j1]=Pix_Data[i][j];
                }
               
             }    
            
         }
int i3,i2;

 void hdgs()
 {
  // countblack1=0;
   for(i3=0;i3<V;i3++)
   { countblack1=0;
      for(i2=0;i2<Hx;i2++)
       {
         if (Pix_Data1[i3][i2] == 0){countblack1++;}
        }
   }
   
 }

void GetBlackEndParam()  //获取黑线截止行，十字路口时用到！
{
	unsigned char LEndFlag = 0;
	unsigned char MEndFlag = 0;
	unsigned char REndFlag = 0;
	int i=0;
        int g_Derict;
	BlackEndL = 0;
	BlackEndM = 0;
	BlackEndR = 0;
	for (i = V-1; i >= 0 ; i--)
	{
		if(ThresholdData[i][Hx/2] == 255 && !MEndFlag ){
			BlackEndM++;
		}
		else if(i > 1 && ThresholdData[i-1][Hx/2] == 0 && ThresholdData[i-2][Hx/2] == 255){
			MEndFlag = 1;
		}
		if(ThresholdData[i][Hx/4] == 0 && !LEndFlag ){
			BlackEndL++;
		}
		else if(i > 1 && ThresholdData[i-1][Hx/4] == 0 && ThresholdData[i-2][Hx/4] == 0){
			LEndFlag = 1;
		}
		if(ThresholdData[i][Hx*3/4] == 255 && !REndFlag ){
			BlackEndR++;
		}
		else if(i > 1 && ThresholdData[i-1][Hx*3/4] == 0 && ThresholdData[i-2][Hx*3/4] == 0){
			REndFlag = 1;
		}
	}
        BlackEndL=BlackEndL+23;
	BlackEndMax = MAX(BlackEndL,BlackEndM);
	BlackEndMax = MAX(BlackEndMax,BlackEndR);
	BlackEndLMR = BlackEndL + BlackEndM + BlackEndR;
//		//判断十字左右倾
	/*if (BlackEndMax == BlackEndL)
	{
		//g_Derict = 1;//左倾
	}
	else if (BlackEndMax == BlackEndR)
	{
		//g_Derict = 2;//右倾
	}
	else if (BlackEndMax == BlackEndM)
	{
		if (fabss(BlackEndL-BlackEndR) < 5)
		{
			//g_Derict = 0;//正十字
		}
		else if (BlackEndL > BlackEndR)
		{
			//g_Derict = 1;//左
		}
		else
		{
			//g_Derict = 2;//右
		}
	}*/
//	str.Format("BlackEndL:%d\r\nBlackEndR:%d\r\n BlackEndM:%d\r\n BlackEndLMR:%d \r\n",BlackEndL,BlackEndR,BlackEndM,BlackEndLMR);
//	fprintf(pfile,str);
}


void GetExcursionLine()
{
	int i=0,j=0;
	int CountBlack = 0;
	int CountWhite = 0;

	//每行头尾遍历行指针
	int pLeft = 0;
        int pRight = 69;
	
	BlackRow=0;
	WhiteRow=0;
	WhiteRow1=0;	
	for (i = 0;i < V;i++)
	{
		//初始化遍历指针
		pLeft = 0;
		pRight =69;
		LeftBlack[i] = pLeft;
		RightBlack[i] = pRight;
		CountBlack = 0;
		CountWhite = 0;

		//两条黑线边缘提取
		for (j = 0;j <= Hx/2;j++)
		{
			// 从左到右搜索边缘
			if (LeftBlack[i] == 0){
				     if (ThresholdData[i][pLeft] == 0){CountBlack++;}
				        else{CountWhite++;}
				// 未找到左边缘则寻找
				     if (ThresholdData[i][pLeft] != ThresholdData[i][pLeft+3])
						{
					//确认检测到边缘
					       if (ThresholdData[i][pLeft+1] != ThresholdData[i][pLeft+4]
						&& ThresholdData[i][pLeft+2] != ThresholdData[i][pLeft+5])
					    {
						//找到左边缘
						          LeftBlack[i] = pLeft + 2;
						//如果右边缘也找到则退出
						if (RightBlack[i]!=Hx-1)
							    {
							        break;
						      }
					     }
					else
						  {
						   pLeft++;
					    }
				}
				else{
					pLeft++;
				}
			}// if结束--从左到右搜索边缘
			
			//从右到左搜索边缘
			if (RightBlack[i] == Hx - 1){
				if (ThresholdData[i][pRight] == 0){CountBlack++;}
				else{CountWhite++;}

				if (ThresholdData[i][pRight] != ThresholdData[i][pRight-3]){
					//确认检测到边缘
					if (ThresholdData[i][pRight-1] != ThresholdData[i][pRight-4]
						&& ThresholdData[i][pRight-2] !=ThresholdData[i][pRight-5]){
						// 找到右边缘
						RightBlack[i] = pRight - 2;
						// 如果左边缘也找到则退出
						if (LeftBlack[i]!=0){
							break;
						}
					}
					else{
						pRight--;
					}
				}
				else{
					pRight--;
				}
			}// if结束--从右到左搜索边缘
			// 无边缘则跳出
			if (pLeft >= pRight){
				LeftBlack[i] = RightBlack[i] = pLeft;
				//判断记录全黑行
				if(CountBlack >=Hx- 5) //黑点的列数！！！
					{
					CountBlack = 0;
					LineType[i] = 1;//全黑行标志
					BlackRow++;
				  }
				else if(CountWhite >= Hx - 5 )
				{
					CountWhite = 0;
					WhiteRow1++;
					if (i < V-5 && i > 35)//近端10行不检测全白行，除掉前端盲区
					{
						WhiteRow++;
						LineType[i] = 2;//全白行标志
					}
					else
					{
						LineType[i] = 0;	
					}
				}
				else
				{
					LineType[i] = 0;	
				}
				break;
			}
		}//for结束--每行黑线提取结束
	}//for结束-- 行扫描完
}
float EPerCount=0.0;
//从偏差线提取偏差
void GetEPerCount()
{
	unsigned char i=0,j=0;
	//中心点跳变计数
	unsigned char TripPointCount = 0;
	// 连续中心线点数小于该数字则过滤掉
	unsigned char FilterNumber = 8;
	// 跳变长度设定
	unsigned char TripLen = 5;//5


	Excursion = 0;
	ValidLineCount=0;
	ValidExcursionCount=0;

	for (i = 0;i < V;i++)
	{
		TripPointPos[i] = 0;	
	}
	//根据跳变分段
	for (i = 0;i < V;i++)
	{
		// 单边缘情况直接取边缘
		if (fabss(RightBlack[i] - LeftBlack[i]) < 5 )     //列数相减
			{
			  BlackLineData[i] = LeftBlack[i];
		  }
		else{
			//正常两边黑线提取中心线
			BlackLineData[i] = LeftBlack[i] + (RightBlack[i] - LeftBlack[i])/2;
		}
		// 相邻中心点做差，找出中心线跳变处,进行分段处理
		if (i > 0)
		{
			SubValue[i] = BlackLineData[i] - BlackLineData[i-1];
			//跳变判断
			if (fabss(SubValue[i]) > TripLen){
				//记录跳变位置（行标）
				TripPointPos[TripPointCount] = i;
				TripPointCount++;	
			}
		}	
	}//for结束 
	if (TripPointCount == 0)
	{
		for (i = 0;i < V;i++)
		{
			ValidLineCount++;
			if(i > 30 && i < V)
			{
				ValidExcursionCount++;
				Excursion += (BlackLineData[i-1]-BlackLineData[i]);
			}
		}
	}
	else 
	{	
		if (TripPointPos[0] > FilterNumber)
		{
			for (i = 0;i < TripPointPos[0];i++)
			{
				ValidLineCount++;
				if(i > 30)
				{
					ValidExcursionCount++;
					Excursion += (BlackLineData[i-1]-BlackLineData[i]);
				}
			}
		}
		TripPointPos[TripPointCount] = V;
		for (j = 0;j < TripPointCount;j++)
		{
			if (TripPointPos[j+1] - TripPointPos[j] > FilterNumber)
			{
				for (i = TripPointPos[j];i < TripPointPos[j+1];i++)
				{
					ValidLineCount++;
					if(i > TripPointPos[j] && i > 30)
					{
						ValidExcursionCount++;
						Excursion += (BlackLineData[i-1]-BlackLineData[i]);
					}
				}
			}//End if
		}//End for
	}//End else
	if(ValidExcursionCount > 1)
	{
		EPerCount = fabss(Excursion)*1.0/ValidExcursionCount;
	}
	else
	{
		EPerCount = 3;
		Excursion=50;
	}
}

unsigned char ValidLine[V]={0};//1 - 左找到线 2--右找到线  两边都找到线-3 都找不到线-0
int NoValidMax=0;//连续两边找不到线计数

//int RoadWidth[12] = {71,70,69,68,66,65,64,63,62,61,60,59};

//提取图像中心线
void GetLMR()
{	
	int i=0,j=0;
	int temLeft = 0;
	int temRight = 0;
	int pLeft = Hx/2,pRight = Hx/2;
	int bFoundLeft = 0;
	int bFoundRight = 0;
	unsigned char TripLen = 6;//6

	int LeftEnd = 0;
	int RightEnd = 0;
	int MidEnd = 0;

	int MidToBlackCount = 0;


	int NoValidCount = 0;
	NoValidMax=0;

	StableNumbers=0;
	LeftStableNumbers = 0;
	RightStableNumbers = 0;
	
	ValidLineCount1=0;
	ValidLineCount2=0;

	
	for (i = V-1;i >= 0 && !MidEnd;i--)
	{
		// 初始化遍历指针
		if (i < V - 5){
			pLeft = BlackLineData[i+1];
			pRight = BlackLineData[i+1];
		}
		else{
			if(BlackEndL < 5 && BlackEndR > BlackEndM){
				pLeft = Hx*3/4;
				pRight = Hx*3/4;	
			}
			else if(BlackEndR < 5 && BlackEndL > BlackEndM){
				pLeft = Hx/4;
				pRight = Hx/4;
			}else{
				pLeft = Hx/2;
				pRight = Hx/2;
			}
		}


		// 初始化标记
		bFoundLeft = bFoundRight = 0;

		// 两条黑线边缘提取
		for (j = 0;j < Hx;j++)
		{
			// 往左搜索
			if (bFoundLeft == 0 && pLeft > 0 && !LeftEnd){// 未找到左边缘则寻找
				//if (pLeft < 1){break;}
				if (ThresholdData[i][pLeft] == 255 && ThresholdData[i][pLeft-1] == 0){
					// 找到左边缘
					LeftBlack[i] = pLeft-1;
					if (LeftBlack[i] > 0)
					{
						bFoundLeft = 1;
					}
					
					// 如果右边缘也找到则退出
					if (bFoundRight){
						break;
					}
				}
				else{
					pLeft--;
				}
			}// if结束--从左到右搜索边缘
			
			//往右搜索
			if (bFoundRight == 0 && pRight < Hx-1 && !RightEnd){//未找到右边缘则寻找
				//if (pRight > ColumnMax - 3){break;}
				if (ThresholdData[i][pRight] == 255 &&  ThresholdData[i][pRight+1] == 0){
					// 找到右边缘
					RightBlack[i] = pRight + 1;
					if (RightBlack[i] < Hx-1)
					{
						bFoundRight = 1;
					}
					
					// 如果左边缘也找到则退出
					if (bFoundLeft){
						break;
					}
				}
				else{
					pRight++;
				}
			}// if结束
		}//for结束


		if (i < V-6 && !bFoundLeft&&!bFoundRight)
		{
			ValidLine[i] = 0;
			NoValidCount++;
			if (NoValidCount > NoValidMax)
			{
				NoValidMax = NoValidCount;
			}
		}
		else
		{
			NoValidCount=0;
		}

		if(bFoundLeft && bFoundRight)
                {
			ValidLineCount1++;
			ValidLine[i] = 3;
		}else
		{
			if (bFoundLeft )
			{
				ValidLineCount2++;
				ValidLine[i] = 1;
			}
			if(bFoundRight)
			{
				ValidLine[i] = 2;
			}
		}

		if (!bFoundLeft) 
		{ 
			if (i < 30)
			{
				LeftBlack[i] = LeftBlack[i+1] + LeftBlack[i+1] - LeftBlack[i+2]; 
			}
			else
			{			
					LeftBlack[i] = 0; 		
			}
		}
		else if (i < 30 && fabss(LeftBlack[i] - LeftBlack[i+1]) > TripLen  )
		{
			LeftBlack[i] = LeftBlack[i+1] + LeftBlack[i+1] - LeftBlack[i+2];
		}

		if (!bFoundRight)
		{
			if (i < 30)
			{ 
				RightBlack[i] = RightBlack[i+1] + RightBlack[i+1] - RightBlack[i+2];
			}
			else
			{
					RightBlack[i] =Hx-1; 			
				
			}
		}
		else if (i < 30 && fabss(RightBlack[i] - RightBlack[i+1]) > TripLen )
		{
			RightBlack[i] = RightBlack[i+1] + RightBlack[i+1] - RightBlack[i+2];
		}
		
		if (LeftBlack[i] > RightBlack[i])
		{ 
			temRight = temLeft = (LeftBlack[i] + RightBlack[i])/2;
			LeftBlack[i] = temLeft;
			RightBlack[i] = temRight;
		}
		
		if (!LeftEnd){LeftStableNumbers++;}
		if (!RightEnd){RightStableNumbers++;}

		if (LeftEnd && !RightEnd)
		{
			BlackLineData[i] = BlackLineData[i+1] + RightBlack[i+1] - RightBlack[i+2];
		}
		else if (!LeftEnd && RightEnd)
		{
			BlackLineData[i] = BlackLineData[i+1] + LeftBlack[i+1] - LeftBlack[i+2];
		}
		else if (LeftEnd && RightEnd)
		{
			MidEnd = 1;
			break;
		}
		else
		{
			BlackLineData[i] = LeftBlack[i] + (RightBlack[i]-LeftBlack[i] )/2;
		}

		//中心线搜索截止
		if (BlackLineData[i] < 4 || BlackLineData[i] > Hx-4)
		{
			MidEnd = 1;
			break;
		}
		if (i < V-20 && fabss(BlackLineData[i]-BlackLineData[i+1]) > TripLen)
		{
			BlackLineData[i] = BlackLineData[i+1] + BlackLineData[i+1] - BlackLineData[i+2];
		}
		if (ThresholdData[i][BlackLineData[i]] == 0)
		{
			MidToBlackCount++;
			if (MidToBlackCount >= 2)
			{
				MidEnd = 1;
			}
		}
		else
		{
			MidToBlackCount = 0;
		}
		if (!MidEnd)
		{
			StableNumbers++;
		}
		if (ValidLineCount1 < 4 && i < 20)
		{
			MidEnd = 1;
		}
	}//for结束-- 行扫描完


}

void GetFinalMidLine()
{
	int i = 0,MinStable = 0;
	MinStable = Min(StableNumbers,LeftStableNumbers);
	MinStable = Min(MinStable,RightStableNumbers);
	for (i = V-1;i > V-(MinStable-5) ;i--)
	{
		//ThresholdData[i][BlackLineData[i]]=White;
		BlackLineData[i] = LeftBlack[i] + (RightBlack[i]-LeftBlack[i] )/2;
		
		if (BlackLineData[i] >Hx-1){BlackLineData[i] = Hx-1;}
		else if (BlackLineData[i] < 0){BlackLineData[i]=0;}
		
		//ThresholdData[i][BlackLineData[i]]=128;
	}
	if (StableNumbers > MinStable)
	{
		for (i = V-(MinStable-5);i > V - (StableNumbers-5);i--)
		{
			BlackLineData[i] = BlackLineData[i+1] + BlackLineData[i+1] - BlackLineData[i+2];
			if (BlackLineData[i] > Hx-1){BlackLineData[i] = Hx-1;}
			else if (BlackLineData[i] < 0){BlackLineData[i]=0;}
		}
	}
}


int P0_X = 0;
int P0_Y = 0;
int P1_X = 0;
int P1_Y = 0;
int P2_X = 0;
int P2_Y = 0;
float Mid_K1 = 0.0;
float Mid_K2 = 0.0;


void LAverageFilter()
{
	unsigned char i = 0;
	unsigned char j = 0;
	int sum = 0;
	for (i = V-1;i > V-(LeftStableNumbers-5);i--)
	{
		sum = 0;
		for (j = 0;j < 5;j++)
		{
			sum += LeftBlack[i-j];
		}
		LeftBlack[i] = sum/5;
	}
	P1_X = LeftBlack[V-(LeftStableNumbers-6)];
	P1_Y = V-(LeftStableNumbers-6);
}


void RAverageFilter()
{
	unsigned char i = 0;
	unsigned char j = 0;
	int sum = 0;
	for (i = V-1;i > V-(RightStableNumbers-5);i--)
	{
		sum = 0;
		for (j = 0;j < 5;j++)
		{
			sum += RightBlack[i-j];
		}
		RightBlack[i] = sum/5;
	}
	P2_X = RightBlack[V-(RightStableNumbers-6)];
	P2_Y = V-(RightStableNumbers-6);

}


void AverageFilter()
{
	unsigned char i = 0;
	unsigned char j = 0;
	int sum = 0;
	for (i = V-1;i > V-(StableNumbers-5-5);i--)
	{
		sum = 0;
		for (j = 0;j < 5;j++)
		{
			sum += BlackLineData[i-j];
		}
		BlackLineData[i] = sum/5;
	}
	P0_X = BlackLineData[V-1];
	P0_Y = V-1;

	Mid_K1 = fabss(P0_X-P1_X)*1.0/fabss(P0_Y-P1_Y);
	Mid_K2 = fabss(P0_X-P2_X)*1.0/fabss(P0_Y-P2_Y);

}

//中心线补偿处理
void MidLineCompensate()
{
	int i=0,icount=0,j=0;
	int CompensateData = 0;
	
	int sum = 0;
	float avg =0.0;
	int tem = 1;

	CompensateCount=0;

	for (i = V-2,icount=0;i > V-(StableNumbers-10);i--,icount++)
	{
		sum += (BlackLineData[i]-BlackLineData[i+1]);
	}
	avg = sum*1.0 / icount;
	if (avg < 0)
	{
		tem = -1;
		avg = (-1)*avg;
	}
	if (avg > 1.0)
	{
		CompensateData = 4;
	}else if (avg > 0.55)
	{
		CompensateData = 3;
	}
	else if (avg > 0.25)
	{
		CompensateData = 2;
	}else
	{
		CompensateData = 0;
	}
	CompensateData = CompensateData*tem;


	for (i = V - (StableNumbers-10);i > 0;i--)
	{
		BlackLineData[i] = BlackLineData[i+1] + CompensateData;//BlackLineData[i+1] + BlackLineData[i+1]-BlackLineData[i+2];
		
		CompensateCount++;
		if (ThresholdData[i][BlackLineData[i]] == 0 || BlackLineData[i] < 2 || BlackLineData[i] > Hx -2)
		{
			break;
		}
		sum = 0;
		for (j = V-2,icount=0;j > i;j--,icount++)
		{
			sum += (BlackLineData[j]-BlackLineData[j+1]);
		}
		avg = sum*1.0 / icount;
		if (avg < 0)
		{
			tem = -1;
			avg = (-1)*avg;
		}
		if (avg > 1.0)
		{
			CompensateData = 4;
		}else if (avg > 0.55)
		{
			CompensateData = 3;
		}
		else if (avg > 0.25)
		{
			CompensateData =2;
		}else
		{
			CompensateData = 0;
		}
		CompensateData = CompensateData*tem;
	}
}

float GetSteerError(unsigned char start,unsigned char end,float midpos)
{
	unsigned char i=0;
	unsigned char iCount=0;
	unsigned int  Black_Sum=0;
	float TemError = 0.0;
	for(i = start,iCount = 0; i < end; i++,iCount++)    
	{	
		Black_Sum += BlackLineData[i];
	}
	TemError = 	Black_Sum*1.0/iCount - midpos;
        if(TemError > 40)
	{
		TemError = 40;
	}
	if(TemError < -40)
	{
		TemError = -40;	
	}
	return TemError;
}

//获取中心线方差
void GetMidLineVariance()
{
	unsigned char i=0;
	unsigned char iCount=0;
	unsigned int  Black_Sum=0;
	float aver=0.0;
	int end = V - (StableNumbers - 5);

	MidLineExcursion = 0;

	for(i=V-2,iCount=0; i>end; i--,iCount++)    
	{	
		Black_Sum += BlackLineData[i];
		MidLineExcursion = MidLineExcursion + BlackLineData[i] - BlackLineData[i+1];
	}	
	aver = Black_Sum*1.0 / iCount;
	MidLineVariance = 0.0;
	for (i = V-2;i > end;i--)
	{
		MidLineVariance+=(aver-BlackLineData[i])*(aver-BlackLineData[i]);
	}
	MidLineVariance = MidLineVariance*1.0 / iCount;
}

//获取特殊中心线偏差 ，StableNumbers要大于2
void GetSpecialError()
{
	unsigned char i=0;
	int end = V - StableNumbers;
	
	MidLineExcursion = 0;
	for (i = V-1;i > end ;i--)
	{
		BlackLineData[i] = LeftBlack[i] + (RightBlack[i]-LeftBlack[i] )/2;		
	}
	for(i=V-2; i>end; i--)    
	{	
		MidLineExcursion = MidLineExcursion + BlackLineData[i] - BlackLineData[i+1];		
	}
	//特殊情况给中心线方差极大
	MidLineVariance = 300;
	//特殊情况给中心线偏差极大
	if(MidLineExcursion > 0)
	{
		MidLineExcursion = 40;
	}
	else if(MidLineExcursion < 0)
	{
		MidLineExcursion = -40;	       
	}
}

int TopE1=0;//中心线上半部分偏差
int TopE2=0;//中心线下半部分偏差
int TopLen1=0;//中心线上半部分偏差
int TopLen2=0;//中心线上半部分偏差 
int SubBasePoint = 0;//中心线下半部分距基准点的最大偏差

void GetSectionParam()   //对中心线分段提取偏差
{
	int TotalPoint = StableNumbers - 10;
	int icount = 0;
	int i = 0;
	int BasePoint = BlackLineData[V-1];
	SubBasePoint = 0;
	TopE1=0;
	TopE2=0;
	TopLen1=0;
	TopLen2=0;
	
	for (i=V-(StableNumbers-10),icount=1;i < V-2;i++,icount++)
	{
		if (icount < TotalPoint/2)
		{
			TopE1 += (BlackLineData[i]-BlackLineData[i+1]);
			TopLen1++;
		}
		else 
		{
			TopE2 += (BlackLineData[i]-BlackLineData[i+1]);
			TopLen2++;
			if (fabss(BlackLineData[i]-BasePoint) > fabss(SubBasePoint))
			{
				SubBasePoint = BlackLineData[i]-BasePoint;
			}
		}
	}
}


int StoreFlag = 0;
signed char TemMidLineData[V];//提取黑线值数据2

void StoreMidLine()
{
	int i = 0;
	for(i = 0;i < V;i++)	
	{
		TemMidLineData[i] = BlackLineData[i];	
	}
}


void UseTemMidLine()
{
	int i = 0;
	for(i = 0;i < V;i++)	
	{
		BlackLineData[i] = TemMidLineData[i];	
	}	
}




void GetCrossingMidLine()
{
	int i = 0,j=0;
	//??DDí·?2±éàúDD????
	int pLeft = Hx/2,pRight = Hx/2;
	int bFoundLeft = 0;
	int bFoundRight = 0;
	int temLeft = 0,temRight = 0,temi=0;
	int temBasePos = 0;

	unsigned char EndFlag = 0;
	int LCount=0;
	int RCount=0;
	unsigned char bFoundFlag=0;


	CrossingStable=0;
	for (i = 0;i < V;i++)
	{
		ValidLineR[i] = 0;
		ValidLineL[i] = 0;
	}
       
        
     // g_Derict = 2;
        
	//?D??ê?×?×óóò??
	if (BlackEndMax == BlackEndL)////////////////////////////BlackEndMax == BlackEndL
	{
		g_Derict = 1;//×ó??
	}
	else if (BlackEndMax == BlackEndR)
	{
		g_Derict = 2;//óò??
	}
	else if (BlackEndMax == BlackEndM)
	{
		if (fabss(BlackEndL-BlackEndR) < 5)//////////////////////////////////5
		{
			g_Derict = 0;//?yê?×?
		}
		else if (BlackEndL > BlackEndR)/////////////////////////////////BlackEndL > BlackEndR
		{
			g_Derict = 1;///////////////////////////////////////////g_Derict = 1;
		}
		else
		{
			g_Derict = 2;///////////////////////////////////////////g_Derict = 2;
		}
	}

	//???÷ê?×?á?±???
	for (i = V-1; i > 0;i--)
	{
		if (!EndFlag)
		{
			CrossingStable++;
		}
		//?ü??15DD′ó?D??íùá?±????÷
		if (i > V - 15 )
		{
			// 3?ê??ˉ±éàú????
			pLeft = Hx/2;
			pRight = Hx/2;
		}
		else 
		{
			// 3?ê??ˉ±éàú????
			pLeft = g_BasePos;
			pRight = g_BasePos;
		}
// 		str.Format("%d pLeft:%d  pRight:%d\r\n",i,pLeft,pRight);
// 		fprintf(pfile,str);
		// 3?ê??ˉ??×?±ê??
		bFoundLeft = bFoundRight = 0;
		for (j = 0;j < Hx;j++)
		{
			// íù×ó???÷
			if (bFoundLeft == 0 && pLeft > 0){// ?′?òμ?×ó±??μ?ò?°?ò
				//if (pLeft < 1){break;}
				if ((ThresholdData[i][pLeft] == 255 && ThresholdData[i][pLeft-1] == 0) || pLeft == 1){
//					ThresholdData[i][Left0[i]]=255;
					// ?òμ?×ó±??μ
					LeftBlack[i] = pLeft-1;
					bFoundLeft = 1;
//					ThresholdData[i][LeftBlack[i]]=100;
					if (LeftBlack[i] > 0)
					{
						ValidLineL[i]=1;
						LCount=0;
					}
					else
					{
						LCount++;
						if (LCount > NoValidLMax)
						{
							//×ó±?á?D??a??×üêy
							NoValidLMax=LCount;
						}
					}
					// è?1?óò±??μò2?òμ??òí?3?
					if (bFoundRight){
						break;
					}
				}
				else{
					pLeft--;
				}
			}// if?áê?--′ó×óμ?óò???÷±??μ
			
			//íùóò???÷
			if (bFoundRight == 0 && pRight < Hx-1 ){//?′?òμ?óò±??μ?ò?°?ò
				//if (pRight > H - 3){break;}
				if ((ThresholdData[i][pRight] == 255 && ThresholdData[i][pRight+1] == 0) || pRight == Hx-2){
//					ThresholdData[i][RightBlack[i]]=255;
					// ?òμ?óò±??μ
					RightBlack[i] = pRight + 1;
//					ThresholdData[i][RightBlack[i]]=100;
					bFoundRight = 1;
					if (RightBlack[i] < Hx-1)
					{
						ValidLineR[i] = 1;
						RCount=0;
					}else
					{
						RCount++;
						if (RCount > NoValidRMax)
						{
							NoValidRMax=RCount;
						}
					}
					
					// è?1?×ó±??μò2?òμ??òí?3?
					if (bFoundLeft){
						break;
					}
				}
				else{
					pRight++;
				}
			}// if?áê?
		}//for?áê?

		//×ó±??ò2?μ?±??μ??0
		if (!bFoundLeft)
		{
			LeftBlack[i]=0;
			//ThresholdData[i][LeftBlack[i]]=100;
		}
		//óò±??ò2?μ?±??μ??×?′ó?μ
		if (!bFoundRight)
		{
			RightBlack[i]=Hx-1;
			//ThresholdData[i][RightBlack[i]]=100;
		}

//		if (i < V-2 && fabss(LeftBlack[i] - LeftBlack[i+1]) > TripLen  )
//		{
//			ValidLine[i]=0;;
//		}
//		if (i < V-2 && fabss(RightBlack[i] - RightBlack[i+1]) > TripLen )
//		{
//			ValidLine[i]=0;
//		}

		if ( i < V - 15)
		{
			//×ó??μ????÷?é??￡??DD????e?￥íù×ó
			if (g_Derict == 1)
			{
				//??μ?μ??DD?μ???????×ó·??ò??à??-?ùμ?????￡?2￠?òóDD§￡?×÷?aD?μ????÷?eμ?
				if (LeftBlack[i] + (RightBlack[i]-LeftBlack[i] )/2 < g_BasePos ///////////////////////////////////2
					&& fabss((LeftBlack[i] - LeftBlack[i+1])) < 3 ////////////////////////////////////////////3
					&& fabss((RightBlack[i] - RightBlack[i+1])) < 3)/////////////////////////////////////////3
				{
					temBasePos = LeftBlack[i] + (RightBlack[i]-LeftBlack[i] )/2;
					if (fabss(temBasePos-g_BasePos)<20)///////////////////////////////////////////////////////20
					{
						g_BasePos = temBasePos;
						if (g_BasePos < 2)/////////////////////////////////////////////////////////////////2
						{
							EndFlag = 1;
						}
					}
				}
				//??μ?μ??DD?μ???????óò·??ò??à??-?ùμ?????￡??ò??óò±??μ?÷ê?íùóò￡??ò′óD????÷?eμ?
				else if (LeftBlack[i] + (RightBlack[i]-LeftBlack[i] )/2 > g_BasePos || RightBlack[i] > RightBlack[i+1]+2)
				{
					temLeft = 0;
					temRight = 0;
					bFoundFlag=0;
					//è·?¨D?μ????÷?eμ?
// 					str.Format("???÷?eμ?:g_BasePos %d  \r\n",g_BasePos);
// 					fprintf(pfile,str);
					for (temi = 1;temi < Hx-1;temi++)
					{
						if (ThresholdData[i][temi] == 255 && ThresholdData[i][temi+1] == 255 && temLeft == 0)
						{
							temLeft = temi;
						}
						if (temLeft != 0)
						{
							if (ThresholdData[i][temi]== 0 && ThresholdData[i][temi+1] == 0 && ThresholdData[i][temi+5] == 0)
							{
								temRight = temi;
								bFoundFlag=1;
								break;
							}
						}
					}
					if(bFoundFlag && temLeft + (temRight-temLeft)/2 < g_BasePos)
					{
						temBasePos = temLeft + (temRight-temLeft)/2;
						g_BasePos = temBasePos;
						if (g_BasePos < 3)//////////////////////////////////////////////////////////////////3
						{
							//D?μ????÷?eμ?ò??-μ?×?×ó±?á?￡??TDè?ù???÷á?
							EndFlag = 1;
						}
					}
// 					str.Format("?òμ?D?μ????÷?eμ?:g_BasePos %d  \r\n",g_BasePos);
// 					fprintf(pfile,str);
				}
			}
			
			//ê?×?óò??μ??é??
			if (g_Derict == 2 )
			{
				//??μ?μ??DD?μ???????óò·??ò??à??-?ùμ?????￡?2￠?òóDD§￡?×÷?aD?μ????÷?eμ?
				if (LeftBlack[i] + (RightBlack[i]-LeftBlack[i] )/2 > g_BasePos 
					&& fabss((LeftBlack[i] - LeftBlack[i+1])) < 3 ///////////////////////////////////////////////3
					&& fabss((RightBlack[i] - RightBlack[i+1])) < 3)//////////////////////////////////////////////3
				{
					temBasePos = LeftBlack[i] + (RightBlack[i]-LeftBlack[i] )/2;
					if (fabss(temBasePos-g_BasePos)<20)////////////////////////////////////////////////////////////20
					{
						g_BasePos = temBasePos;
						if (g_BasePos > Hx-4)
						{
							EndFlag = 1;
						}
					}
				}
				//??μ?μ??DD?μ???????×ó·??ò??à??-?ùμ?????￡??ò??×ó±??μ?÷ê?íù×ó￡??ò′óD????÷?eμ?
				else if (LeftBlack[i] + (RightBlack[i]-LeftBlack[i] )/2 < g_BasePos || LeftBlack[i] < LeftBlack[i+1]-2)
				{
					temLeft = 0;
					temRight = 0;
					bFoundFlag = 0;
					//è·?¨D?μ????÷?eμ?
					for (temi = Hx-1;temi > 1;temi--)
					{
						if (ThresholdData[i][temi] == 255 && ThresholdData[i][temi-1] == 255 && temRight == 0)
						{
							temRight = temi;
						}
						if (temRight != 0)
						{
							if (ThresholdData[i][temi]== 0 && ThresholdData[i][temi-1]== 0 && ThresholdData[i][temi-5] == 0 )
							{
								temLeft = temi;
								bFoundFlag=1;
								break;
							}
						}
					}
					if(bFoundFlag && temLeft + (temRight-temLeft)/2 > g_BasePos)
					{
						temBasePos = temLeft + (temRight-temLeft)/2;
						g_BasePos = temBasePos;
						if (g_BasePos > Hx-3)
						{
							EndFlag = 1;
						}
					}
// 					str.Format("?òμ?D?μ????÷?eμ?:g_BasePos %d  \r\n",g_BasePos);
// 					fprintf(pfile,str);
				}
			}
		}
	}

// 	str.Format("CrossingStable:%d\r\n",CrossingStable);
// 	fprintf(pfile,str);
// 	str.Format("NoValidLMax:%d NoValidRMax:%d\r\n",NoValidLMax,NoValidRMax);
// 	fprintf(pfile,str);
}

//?ú??á?μ????ao?
void CommonRectificate(int data[],unsigned char begin,unsigned char end)
{
	int MidPos = 0;
	if (end > V-1)
	{
		end = V-1;
	}
	if (begin == end)
	{
//		ThresholdData[begin][data[begin]]=255;

		data[begin] = (data[begin-1]+data[begin+1])/2;
// 		str.Format("213￥%d:%d  \r\n",n++,begin);
// 		fprintf(pfile,str);
//		ThresholdData[begin][data[begin]]=100;

//		ThresholdData[begin][BlackLineData[begin]]=255;

		BlackLineData[begin] =  LeftBlack[begin] + (RightBlack[begin]-LeftBlack[begin])/2;
		
//		ThresholdData[begin][BlackLineData[begin]]=128;

	}
	else if(begin < end)
	{
		MidPos = (begin+end)/2;	
//		ThresholdData[MidPos][data[MidPos]]=255;
		data[MidPos] = (data[begin]+data[end])/2;
// 		str.Format("213￥%d:%d  \r\n",n++,MidPos);
// 		fprintf(pfile,str);
//		ThresholdData[MidPos][data[MidPos]]=100;

//		ThresholdData[MidPos][data[MidPos]]=100;
//		ThresholdData[MidPos][BlackLineData[MidPos]]=255;
		BlackLineData[MidPos] =  LeftBlack[MidPos] + (RightBlack[MidPos]-LeftBlack[MidPos])/2;	
//		ThresholdData[MidPos][BlackLineData[MidPos]]=128;
		
		if (begin+1 < MidPos)
		{
			CommonRectificate(data,begin,MidPos);
		}
		if (MidPos+1 < end)
		{
			CommonRectificate(data,MidPos,end);
		}
	}
}


void CrossingMidFilter()
{
	int i = 0,j=0,MidEnd = 0;
	unsigned char MidToBlackCount = 0;
	int sum = 0;
	StableNumbers2 = 0;

	for (i = V-1;i >= 0 && !MidEnd;i--)
	{
		BlackLineData[i] = LeftBlack[i] + (RightBlack[i]-LeftBlack[i] )/2;
		//ê?×??DD????è?¨DD	
		StableNumbers2++;
		//?DD??????÷???1
		if (BlackLineData[i] < 4 || BlackLineData[i] > Hx-4)
		{
			MidEnd = 1;
			break;
		}
		if (i < V-5 && fabss(BlackLineData[i]-BlackLineData[i+1]) > 3)
		{
			BlackLineData[i] = BlackLineData[i+1] + BlackLineData[i+1] - BlackLineData[i+2];
		}
		if (ThresholdData[i][BlackLineData[i]] == 0)
		{
			MidToBlackCount++;
			if (MidToBlackCount >= 3)
			{
				//?DD?????μ?oúμ?á?
				MidEnd = 1;
			}
		}
//		else
//		{
//			MidToBlackCount = 0;
//		}
	}

	for (i = V-1;i > V-(StableNumbers2-5);i--)
	{
		sum = 0;
		for (j = 0;j < 5;j++)
		{
			sum += BlackLineData[i-j];
		}
		BlackLineData[i] = sum/5;
		
		//ThresholdData[i][BlackLineData[i]]=128;
	}
}

//?yê?×?μ??é??′|àí
void SCProcessing()
{
	int i = 0;
	unsigned char startPos=0,endPos=0;
        int temCount=0,countMax=0,temPos=0;
	unsigned char ProcessFlag=0;
	
	//ì?±???êy
	unsigned char TripPointCount = 0;
	unsigned char TripLen = 2;/////////////////////////////////////////////////////////////////////////2
	//è?ê?×?μ?±??μ??
	GetCrossingMidLine();

	for (i = 0;i < V;i++)
	{
		TripPointPos[i] = 0;	
	}
	TripPointCount=0;
	//?ù?Yì?±?·???
	for (i = 0;i < V;i++)
	{
		// ?àáú?DD?μ?×?2?￡??ò3??DD???ì?±?′|,??DD·???′|àí
		if (i > 0){
			SubValue[i] = LeftBlack[i] - LeftBlack[i-1];
			//ì?±??D??
			if (fabss(SubValue[i]) > TripLen || LeftBlack[i] < 2){
				//????ì?±?????￡¨DD±ê￡?
				TripPointPos[TripPointCount] = i;
				TripPointCount++;	
			}
		}	
	}//for?áê? 
	TripPointPos[TripPointCount] = V;
	temCount = 0;
	countMax=0;
	temPos = TripPointPos[0]-1;
	startPos = temPos;
	endPos = 0;
	for (i = 1;i < TripPointCount;i++)
	{
          if (TripPointPos[i]-TripPointPos[i-1] < 3)
		{
			temCount++;
			if (temCount > countMax)
			{
				countMax = temCount;
				startPos = temPos;
				endPos = TripPointPos[i]+1;
			}
		}
		else
		{
			temPos = TripPointPos[i]-1;
			temCount=0;
		}
	}//End for
	if (startPos > 10 /*&& endPos-startPos < 30*/)
	{
		CommonRectificate(&LeftBlack[0],startPos,endPos);
		ProcessFlag=1;
	}

	for (i = 0;i < V;i++)
	{
		TripPointPos[i] = 0;	
	}
	TripPointCount=0;
	//?ù?Yì?±?·???
	for (i = 0;i < V;i++)
	{
		// ?àáú?DD?μ?×?2?￡??ò3??DD???ì?±?′|,??DD·???′|àí
		if (i > 0){
			SubValue[i] = RightBlack[i] - RightBlack[i-1];
			//ì?±??D??
			if (fabss(SubValue[i]) > TripLen || RightBlack[i] > Hx-3){
				//????ì?±?????￡¨DD±ê￡?
				TripPointPos[TripPointCount] = i;
				TripPointCount++;	
			}
		}	
	}//for?áê? 
	TripPointPos[TripPointCount] = V;
	temCount = 0;
	countMax=0;
	
	endPos = 0;
	temPos = TripPointPos[0]-1;	
	startPos = temPos;
	for (i = 0;i < TripPointCount;i++)
	{
		if (TripPointPos[i]-TripPointPos[i-1] < 3)
		{
			temCount++;
			if (temCount > countMax)
			{
				countMax = temCount;
				startPos = temPos;
				endPos = TripPointPos[i]+1;
			}
		}
		else
		{
			temPos = TripPointPos[i]-1;
			temCount=0;
		}
	}//End for
	if (startPos > 10/*&& endPos-startPos < 30*/)
	{
		//?ao?óò±??μ
		CommonRectificate(&RightBlack[0],startPos,endPos);
		ProcessFlag=1;
	}

	if(!ProcessFlag)
	{
		IsCrossing = 0;	
	}
}

void ProcessCrossing()
{
	int i = 0,iStart= V - StableNumbers + 6,iEnd = V-1;
	int iCount=0;
	int TripPos = 0,pos = 0;
	int Count1 = 0,Count2=0;
	int tem0 = 0,tem1=0;
	unsigned char startPos=0,endPos=0;
	unsigned char ProcessFlag = 0;

	LCrossingTripPos = 0;
	RCrossingTripPos = 0;

	bFoundTripPoint = 0;
	if (iStart < 5)//////////////////////////////////////////////////////////////5
	{
		iStart = 5;//////////////////////////////////////////////////////////5
	}
	for (i = iStart;i < iEnd;i++)//////////////////////////////////////////////////////////？///////////H止
	{
		tem0 = RightBlack[i]>Hx-1 ? Hx-1:RightBlack[i];
		tem1 = LeftBlack[i] < 0 ? 0:LeftBlack[i];
		if (tem0 - tem1 > 55)///////////////////////////////////////////////55
		{
			iCount++;
		}
		else
		{
			break;
		}
	}
	if (iCount > 4)/////////////////////////////////////////////////////////////////7
	{
		IsCrossing = 1;
	}
	else
	{
		if (NoValidMax > 4 )//////////////////////8
		{
			IsCrossing = 1;
// 			str.Format("ê?×?′|àí2\r\n");
// 			fprintf(pfile,str);
			if (ValidLine[V-3] == 0)
			{
//				SmoothLeftRight();
				//μ±×??yê?×?′|àí
				SCProcessing();
				return;
			}
			else
			{
				//μ±×??yê?×?′|àí
				SCProcessing();
				return;
			}
		}
		else
		{
			IsCrossing = 0;
			return;
		}
	}

	//?yê?×?
	if(iCount>4&&g_Derict==0 )/////////////////////////////7
	{	
		SCProcessing();
		return;
	}

	//óò???ò×ó±??μì?±?
	//if (iCount > 4 && g_Derict == 2)
          if ( g_Derict == 2)
	{
// 		str.Format("óò??ê?×???×ó±??μ′|àí\r\n");
// 		fprintf(pfile,str);

		i = iStart;
                Count1=0;
		Count2=0;
		while (i < iEnd && LeftBlack[i] - LeftBlack[i+1] == 0){i++;}
// 		str.Format("μ?′?:%d %d\r\n",i,LeftBlack[i]-LeftBlack[i+1]);
// 		fprintf(pfile,str);
		
		if (LeftBlack[i]-LeftBlack[i+1] > 0)
		{
			Count1++;
			i++;
			for (;i < iEnd;i++)
			{
				if (LeftBlack[i]-LeftBlack[i+1] > 0)
				{
					Count1++;
					if (Count2 != 0)
					{
						Count1 = 1;
						Count2 = 0;
					}
				}
				else if (LeftBlack[i]-LeftBlack[i+1] < 0)
				{
					if (Count1 > 2 && TripPos == 0)
					{
						TripPos = i;
					}
					Count2++;
				}
			}
			if (Count1 >2 && Count2 > 2)
			{
				bFoundTripPoint = 1;
				LCrossingTripPos = TripPos;//×ó±??μì?±?μ??òμ?
			}
		}
		else
		{
			Count1++;
			i++;
			for (;i < iEnd;i++)
			{
				if (LeftBlack[i]-LeftBlack[i+1] < 0)
				{
					Count1++;
					if (Count2 != 0)
					{
						Count1 = 1;
						Count2 = 0;
					}
				}
				else if (LeftBlack[i]-LeftBlack[i+1] > 0)
				{						
					if (Count1 >2 && TripPos == 0)
					{
						TripPos = i;
					}
					Count2++;
				}
			}
			if (Count1 >2 && Count2 > 2)
			{
				bFoundTripPoint = 1;
				LCrossingTripPos = TripPos;//×ó±??μì?±?μ??òμ?
			}
		}
	}

	////×ó???òóò±??μì?±?μ?
	if ( g_Derict == 1)
	{
		
		i = iStart;
		Count1=0;
		Count2=0;
		while (i < iEnd && RightBlack[i] - RightBlack[i+1] == 0){i++;}
		if (RightBlack[i]-RightBlack[i+1] > 0)
		{
			Count1++;
			i++;
			for (;i < iEnd;i++)
			{
				if (RightBlack[i]-RightBlack[i+1] > 0)
				{
					Count1++;
					if (Count2 != 0)
					{
						Count1 = 1;
						Count2 = 0;
					}
				}
				else if (RightBlack[i]-RightBlack[i+1] < 0)
				{
					if (Count1 >2 && TripPos == 0)
					{
						TripPos = i;
						
					}
					Count2++;
				}
			}
			if (Count1 > 2 && Count2 > 2)
			{
				bFoundTripPoint = 1;
				RCrossingTripPos = TripPos;//óò±??μì?±?μ??òμ?
			}
		}
		else
		{
			Count1++;
			i++;
			for (;i < iEnd;i++)
			{
				if (RightBlack[i]-RightBlack[i+1] < 0)
				{
					Count1++;
					if (Count2 != 0)
					{
						Count1 = 1;
						Count2 = 0;
					}
				}
				else if (RightBlack[i]-RightBlack[i+1] > 0)
				{
					if (Count1 > 2 && TripPos == 0)
					{
						TripPos = i;
						
					}
					Count2++;
				}
			}
			if (Count1 > 2 && Count2 > 2)
			{
				bFoundTripPoint = 1;
				RCrossingTripPos = TripPos;//óò±??μì?±?μ??òμ?
			}
		}
	}
	if (bFoundTripPoint)
	{
		GetCrossingMidLine();//è?ê?×?μ?±??μ??
		if (g_Derict == 1)//×ó??
		{
			pos = TripPos-8;
			while(pos > 0 && (ValidLineR[pos] == 0 || RightBlack[pos] > RightBlack[TripPos])){pos--;} 			
			if (RightBlack[pos-2] < RightBlack[TripPos]/*&&TripPos-(pos-2)<30*/)//?ao??eμ?o?àí￡??éò???DD?ao?
			{
				//óò±??μ?ao?
				CommonRectificate(&RightBlack[0],pos-2,TripPos);
				ProcessFlag = 1;
			}
			else
			{
				//??DDáíò???·?ê??ao?
				for (i = 0;i < Hx;i++)
				{
					if (ThresholdData[pos-2][i] == 255 && ThresholdData[pos-2][i+1] == 0)
					{
						RightBlack[pos-2]=i;
						break;
					}
				}
				if (RightBlack[pos-2] < RightBlack[TripPos]/*&&TripPos-(pos-2)<30*/)//?ao??eμ?o?àí￡??éò???DD?ao?
				{
					//óò±??μ?ao?
					CommonRectificate(&RightBlack[0],pos-2,TripPos);
					ProcessFlag = 1;
				}
				else if (NoValidLMax > 20/*&&TripPos-(pos-2)<30*/)
				{
					RightBlack[pos-2]=2;
					CommonRectificate(&RightBlack[0],pos-2,TripPos);
					ProcessFlag = 1;

				}
			}
//			
		}
		else if (g_Derict == 2)//óò??
		{
			pos = TripPos-8;
			while(pos > 0 && ValidLineL[pos] == 0 || LeftBlack[pos] < LeftBlack[TripPos]){pos--;}
// 			
				if (LeftBlack[pos-2] > LeftBlack[TripPos]/*&&TripPos-(pos-2)<30*/)
			{
				CommonRectificate(&LeftBlack[0],pos-2,TripPos);
				ProcessFlag = 1;
			}
			else
			{
				//áííaò???·?ê???DD?ao?
				for (i = Hx-1;i > 0;i--)
				{
					if (ThresholdData[pos-2][i] == 255 && ThresholdData[pos-2][i-1] == 0)
					{
						LeftBlack[pos-2]=i;
						break;
					}
				}
				if (LeftBlack[pos-2] > LeftBlack[TripPos] /*&&TripPos-(pos-2)<30*/)
				{
					CommonRectificate(&LeftBlack[0],pos-2,TripPos);
					ProcessFlag = 1;
				}
				else if (NoValidRMax > 20 /*&&TripPos-(pos-2)<30*/)
				{
					LeftBlack[pos-2]=Hx-2;
					CommonRectificate(&LeftBlack[0],pos-2,TripPos);
					ProcessFlag = 1;
				}
			}

//			
		}
	}
	else//?ò2?μ?ì?±?μ?
	{
// 		str.Format("?′?òμ?ì?±?μ?′|àí\r\n");
// 		fprintf(pfile,str);
		GetCrossingMidLine();//è?ê?×?μ?±??μ??
		if (g_Derict == 1)//×ó??
		{
			pos=20;
			while (ValidLineR[pos]==0)
			{
				pos++;
			}
			while(ValidLineR[pos]==1)
			{
				pos++;
			}
			startPos = pos-2;
			pos += 8;
			while (pos < V-1 && (ValidLineR[pos] == 0 || RightBlack[pos] > Hx-3 ))
			{
				pos++;
			}
			endPos = pos+4;
// 
// 			str.Format("startPos:%d. endPos %d\r\n",startPos,endPos);
// 			fprintf(pfile,str);
			//if (RightBlack[pos-1] < 40)
//			if(endPos-startPos<30)
			{
				CommonRectificate(&RightBlack[0],startPos,endPos);
				ProcessFlag = 1;
			}
		}
		else if (g_Derict == 2)
		{
			pos=20;
			while (ValidLineL[pos]==0)
			{
				pos++;
			}
			while(ValidLineL[pos]==1)
			{
				pos++;
			}
			startPos = pos-2;
			pos += 8;
			while (pos < V-1 && (ValidLineL[pos] == 0 || LeftBlack[pos] < 3 ))
			{
				pos++;
			}
			endPos = pos+4;
// 			str.Format("startPos:%d. endPos %d\r\n",startPos,endPos);
// 			fprintf(pfile,str);
			//if (RightBlack[pos-1] < 40)
//			if(endPos-startPos<30)
			{
				CommonRectificate(&LeftBlack[0],startPos,endPos);
				ProcessFlag = 1;
			}
 		}
	}

	if (ProcessFlag == 0)
	{	
		IsCrossing = 0;
		return;
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
void HistoryRoadTypeCount()
{
	int i=0;
	int tem = 0;
	int StraightCount = 0;
	int SmallSCount = 0;
	int BigSCount = 0;
	int BendCount = 0;
	AllStraightCount = 0;
	AllSmallSCount = 0;
	AllBigSCount = 0;
	AllBendCount = 0;
	tem = Rear2;
	for(i = 0;i < ElementCount2;i++)
	{
		if(RoadTypeData2[tem] == 0)////////////////////////////////?±μà
		{
			StraightCount++;
			if(AllStraightCount < StraightCount)
			{
				 AllStraightCount = StraightCount;
			}
		}
		else
		{
			StraightCount=0;
		}

		if(RoadTypeData2[tem] == 0 || RoadTypeData2[tem] == 1)////////////////D?S
		{
			SmallSCount++;
			if(AllSmallSCount < SmallSCount)
			{
				 AllSmallSCount = SmallSCount;
			}
		}
		else
		{
			SmallSCount=0;
		}

		if(RoadTypeData2[tem] == 2)/////////////////////////′óS
		{
			BigSCount++;
			if(AllBigSCount < BigSCount)
			{
				 AllBigSCount = BigSCount;
			}
		}
		else
		{
			BigSCount=0;
		}

		if(RoadTypeData2[tem] == 2 || RoadTypeData2[tem] == 3 )/////////////////////?±í?
		{
			BendCount++;
			if(AllBendCount < BendCount)
			{
				 AllBendCount = BendCount;
			}
		}
		else
		{
			BendCount=0;
		}

		tem = (tem-1+Size2)%Size2;
	}	
} 

void HistoryRTProccess()//Rê?Road
{
	//int TemCount = 0;
	//int pTem = 0;
	//int i = 0;

	if(StandardRoadType && (RoadType == 0 || RoadType == 1))
	{
		//°?èüμàààDí′?è??-?·?óáD?D//±ê×?μ??±μàoíD?S
		RoadTypeData2[Rear2] = RoadType;	
	}
	else
	{
		RoadTypeData2[Rear2] = 2;	
	}

	Rear2 = (Rear2+1)%Size2;
	ElementCount2++;
	if(ElementCount2 > Size2-1)
	{
		ElementCount2=Size2-1;	
	}

	//àúê·èüμàààDíí3??
	HistoryRoadTypeCount();


}
unsigned char IsStraightToBend()
{
        TopE1 = GetSteerError(35,39,35);
        TopE2 = GetSteerError(40,44,35);
	if (fabss(TopE1) < 4 && fabss(TopE2) < 3 )//&& fabss(SubBasePoint) < 3 )// && BlackEndMax < V - 4
	{
		return 1;

	}else
	{
		return 0;
	}
}

int HistoryRoadType[4] = {0};
int g_Head = 0,g_Rear=0;


//int StraightToBendCount = 0;


void RTRecognition()
{
	signed char temRoadType = -1;
	//±ê×?μ?èüμàààDí 3?￠2
	if(fabss(Excursion) > 30)
	{
		if(EPerCount >= 1.2 && BlackEndMax < 15)
		{
			temRoadType = 3;	/////////////////////?±í?
		}
		else if(EPerCount >= 1.0 && StableNumbers <= 28 && BlackEndMax < 22)
		{
			temRoadType = 2;	///////////////////////′óS		   	
		}

	}
	//±ê×?μ?èüμàààDí 1?￠0
      // if(BlackEndMax >= V-1)/////////////////////////////////////////////////////////////改了、
	{
		if(fabss(Excursion)< 5 && MidLineVariance < 2&& EPerCount < 0.1)
		{
			temRoadType = 0;	///////////////////////?±μà
		}
		else if(fabss(Excursion)< 15 && MidLineVariance < 20)
		{
			temRoadType = 1;//////////////////////////////D?S
		}
	}
	//·?±ê×?μ?èüμà′|àí
	if(temRoadType==-1)
	{
		StandardRoadType = 0;
		if(BlackEndMax > 33)
		{
			temRoadType = 1;//////////////////////////////D?S
			if (fabss(TopE1-TopE2) > 27 || Mid_K1 > 1.2 || Mid_K2 > 1.2)
			{		   
				temRoadType = 2;///////////////////////′óS	
				StandardRoadType = 2;
			}
			if(LastRoadType == 103)
			{
				temRoadType = 103;//////////////////////////?±μà×aí?μà
				StraightToBendCount++;
				if(StraightToBendCount > 2)
				{
					temRoadType = 2;///////////////////////′óS
					StraightToBendCount=0;	
				}
			}
		}
		else if (BlackEndMax > 20)
		{
			temRoadType = 2;///////////////////////′óS
		}
		else 
		{
			temRoadType = 3;/////////////////////?±í?
		}
	}
	else             //±ê×?μ?èüμà′|àí
	{
		StandardRoadType = 1;
	}

	RoadType = temRoadType;//èüμàààDí?D??3é1|
       HistoryRTProccess();//??àúê·èüμàààDíD??￠??DD′|àí
	if(IsStraightToBend() )//?D??ê?·?ê??±è?í?
	{
		RoadType = 103;//////////////////////////?±μà×aí?μà
	}
	if(RoadType != 103)
	{
		StraightToBendCount = 0;
	}

	HistoryRoadType[g_Rear] = RoadType;/////////////////////////rearoó??
	g_Rear = (g_Rear+1)%4;
	if(RoadType != HistoryRoadType[g_Head] && HistoryRoadType[g_Head] == HistoryRoadType[(g_Head+1)%4])
	{
		RoadType = HistoryRoadType[g_Head];	
	}
	if((g_Rear+1)%4 == g_Head)
	{
		g_Head	= (g_Head+1)%4;
	}
}

int podao=0,podaocount=0;
extern int enableisstartline;
void SpeedCtrol()
{
      
    if(podao==0)
   {
	SetPoint=speed;
	
    }   
      /*  if(enableisstartline>300) 
        {
          if(CurrentVelocity>15)   podao=3;
          if(podao==3)             {SetPoint=-70;podao=4;}
          if(podao==4)             {if(CurrentVelocity<1) podao=1; }          
          if(podao==1)             SetPoint=-110;
          if(podao==1&&podaocount>380||podao==1&&CurrentVelocity<-100)   podao=2;
          if(podao==2)              SetPoint=45,podaocount=0;
          if(podao==2&&CurrentVelocity>-20) podao=0;
        }  */
	 //if(IsStartLine==1) SetPoint=0;
       
}
/////////////////////////////////
extern u16 stop_count;
extern float Gyro_DIR,Pp;
int StartPos = V - 15;
int EndPos = V-1;
void DirectionCtrol()
{  
        int cc;
        unsigned char i = 0;
	//temerror=0.0;
        int i1,count;
	
	//int MidPos = BlackLineData[V-3] - LeftBlack[V-3]*1.0/2 + (Hx-RightBlack[V-3])*1.0/2;//ê?ê?￡?//////////////易
	// if(temerror>0)
        //{MidPos=MidPos+ ;}
        int MidPos =35;
        /* SetPoint1=SetPoint+50;      
         if(CurrentVelocity<SetPoint1) StartPos=StartPos-1;
         SetPoint=SetPoint1+150;
         if(CurrentVelocity>SetPoint1) StartPos=StartPos+1;*/
	temerror = GetSteerError(StartPos,EndPos,MidPos);
        
          
      // if(Gyro_DIR>0.8)  Pp=15;
	if(IsCrossing)
	{
           for(cc=0;cc<V;cc++)
           {
              RightBlack[cc]=RightBlack[cc]-20;
               LeftBlack[cc]=LeftBlack[cc]+18;
           }
         
            CrossingMidFilter();
                Foresight=3;
		//if(Foresight > StableNumbers+CompensateCount-11)Foresight=StableNumbers+CompensateCount-11;
		StartPos = V - Foresight+1;
		EndPos = V-1;
                MidPos =33;
                temerror = GetSteerError(StartPos,EndPos,MidPos);
           
	}
       
	  if(stop_count<=300) temerror=0;
                
}	 
/////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////






float temp,value,k;
int TemError_Count=0;
int Foresight,StartPos,EndPos,number;

void GetImageParam()
{      
        int i,j,i1;
        float sum,value_buf[N];
        GetBlackEndParam();//获取黑线截止行
	GetExcursionLine();//获取求偏移量线,可以识别出十字交叉道
	GetEPerCount();//求有效偏移量,和平均每列偏移量	
        StoreFlag = 0;
	GetLMR();//提取左边缘、右边缘和中心线
        
	//正常情况下的中心线处理
	if(StableNumbers > 12)
	{       number=StableNumbers;
		LAverageFilter();//左边缘滤波
		RAverageFilter();//右边缘滤波
		GetFinalMidLine();//通过左右边缘取得中心线
		AverageFilter();//中心线滤波处理
		MidLineCompensate();//中心线补偿处
		
		StoreFlag=1;
		StoreMidLine();

		GetSectionParam();//对中心线分段提取偏差
		GetMidLineVariance();//提取中心线方差
               /* if(CurrentVelocity>50) {StartPos=37; EndPos=42;}
                else if(CurrentVelocity>45) {StartPos=39; EndPos=43;}
                else if(CurrentVelocity>40) {StartPos=41; EndPos=45;}
                else {StartPos=45; EndPos=49;}*/
      //   temerror=GetSteerError(StartPos,EndPos,30);   
           
	}
                      ProcessCrossing();
                      RTRecognition();
                      DirectionCtrol();
                      SpeedCtrol();
	
       
       
}



