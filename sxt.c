 #include "ftm.h"
#include "common.h"
#define H 320                            //�ɼ���ͼ�������
#define V 51                              //�ɼ���ͼ�������
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
int ValidLineCount1 = 0; //���Ե�ҵ������ұ��ҵ�
int ValidLineCount2 = 0;//���Ե�ҵ������ұ��ҵ�

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
int LeftBlack[V]; //�����							   
 int RightBlack[V];//�Һ���
 int BlackLineData[V];//������

extern int b;
float temerror=0.0;
int countblack1=0;
float value_buf[N];
unsigned char ThresholdData[V][Hx];  //?t?��?����y?Y��y����		
/////////////////////////////////////////////////////////////////////////////////////
int g_Derict=0; 
int g_BasePos = Hx/2;
unsigned char ValidLineR[V]={0}; //��?��?�̨�������?DD��DD�����??��y����
unsigned char ValidLineL[V]={0};//��?��?�̨�������?DD��DD�����??��y����
unsigned char NoValidLMax = 0;//��?��???2?�̨�������?��?D??a????��y
unsigned char NoValidRMax = 0;//��?��???2?�̨�������?��?D??a????��y
unsigned char CrossingStable = 0;
unsigned char IsCrossing=0;

int LCrossingTripPos = 0;
int RCrossingTripPos = 0;
//?����?��?��?��?��?��?��?
unsigned char bFoundTripPoint = 0;
int startPos=0,endPos=0;
//////////////////////////////////////////////////////////////////////////////////////
signed char RoadType = -1;
signed char LastRoadType = 0;
int ElementCount2 = 0;
//����?2????
int Head2 = 0,Rear2 = 0;
#define Size2 30
//?����D��y����
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
int	BendSpeed = -70;   // ��?�̨�?��?��
int	CommonSpeed = -70;
int	StraightToBendSpeed = -70;

//int SpecialSpeed = -15;

int IncreaseSpeed1 = 0;//3��?���̨�?��?��������y1
int IncreaseSpeed2 = 0;//3��?���̨�?��?��������y2
int IncreaseSpeed3 = 0;//3��?���̨�?��?��������y3
int CanSpeedUp = 0;	 //3��?���̨�����?����1?��
int CanFullSpeed = 0;//?����??����1?��
unsigned char StraightFS=19;//?���̨�?��?��//10
unsigned char SmallSFS=19;//D?s?��?��
unsigned char BigFS=19;//�䨮S?��?��6
unsigned char BendFS=19;//?����??��?��
unsigned char CommonFS=19;//??����?��?��
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
int otsu (unsigned char *image, int rows, int cols, int x0, int y0, int dx, int dy, int vvv)//�ü�Ȩƽ��ֵ����ֵ
{       
	unsigned char *np; // ͼ��ָ��
	int thresholdValue=1; // ��ֵ
	int ihist[256] = {0}; // ͼ��ֱ��ͼ��256����

	int i, j, k; // various counters
	int n, n1, n2, gmin, gmax;
	double m1, m2, sum, csum, fmax, sb;

	//gmin=255; gmax=0;//ͼ����ֵ��Χ���ó�ʼ
	gmin=200; gmax=60;
	// ����ֱ��ͼ
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
	   sum += (double) k * (double) ihist[k]; /* x*f(x) ������*/
	   n += ihist[k]; /* f(x) ���� */
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





//�����ȡ��ֵ�˲�
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

void GetBlackEndParam()  //��ȡ���߽�ֹ�У�ʮ��·��ʱ�õ���
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
//		//�ж�ʮ��������
	/*if (BlackEndMax == BlackEndL)
	{
		//g_Derict = 1;//����
	}
	else if (BlackEndMax == BlackEndR)
	{
		//g_Derict = 2;//����
	}
	else if (BlackEndMax == BlackEndM)
	{
		if (fabss(BlackEndL-BlackEndR) < 5)
		{
			//g_Derict = 0;//��ʮ��
		}
		else if (BlackEndL > BlackEndR)
		{
			//g_Derict = 1;//��
		}
		else
		{
			//g_Derict = 2;//��
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

	//ÿ��ͷβ������ָ��
	int pLeft = 0;
        int pRight = 69;
	
	BlackRow=0;
	WhiteRow=0;
	WhiteRow1=0;	
	for (i = 0;i < V;i++)
	{
		//��ʼ������ָ��
		pLeft = 0;
		pRight =69;
		LeftBlack[i] = pLeft;
		RightBlack[i] = pRight;
		CountBlack = 0;
		CountWhite = 0;

		//�������߱�Ե��ȡ
		for (j = 0;j <= Hx/2;j++)
		{
			// ������������Ե
			if (LeftBlack[i] == 0){
				     if (ThresholdData[i][pLeft] == 0){CountBlack++;}
				        else{CountWhite++;}
				// δ�ҵ����Ե��Ѱ��
				     if (ThresholdData[i][pLeft] != ThresholdData[i][pLeft+3])
						{
					//ȷ�ϼ�⵽��Ե
					       if (ThresholdData[i][pLeft+1] != ThresholdData[i][pLeft+4]
						&& ThresholdData[i][pLeft+2] != ThresholdData[i][pLeft+5])
					    {
						//�ҵ����Ե
						          LeftBlack[i] = pLeft + 2;
						//����ұ�ԵҲ�ҵ����˳�
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
			}// if����--������������Ե
			
			//���ҵ���������Ե
			if (RightBlack[i] == Hx - 1){
				if (ThresholdData[i][pRight] == 0){CountBlack++;}
				else{CountWhite++;}

				if (ThresholdData[i][pRight] != ThresholdData[i][pRight-3]){
					//ȷ�ϼ�⵽��Ե
					if (ThresholdData[i][pRight-1] != ThresholdData[i][pRight-4]
						&& ThresholdData[i][pRight-2] !=ThresholdData[i][pRight-5]){
						// �ҵ��ұ�Ե
						RightBlack[i] = pRight - 2;
						// ������ԵҲ�ҵ����˳�
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
			}// if����--���ҵ���������Ե
			// �ޱ�Ե������
			if (pLeft >= pRight){
				LeftBlack[i] = RightBlack[i] = pLeft;
				//�жϼ�¼ȫ����
				if(CountBlack >=Hx- 5) //�ڵ������������
					{
					CountBlack = 0;
					LineType[i] = 1;//ȫ���б�־
					BlackRow++;
				  }
				else if(CountWhite >= Hx - 5 )
				{
					CountWhite = 0;
					WhiteRow1++;
					if (i < V-5 && i > 35)//����10�в����ȫ���У�����ǰ��ä��
					{
						WhiteRow++;
						LineType[i] = 2;//ȫ���б�־
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
		}//for����--ÿ�к�����ȡ����
	}//for����-- ��ɨ����
}
float EPerCount=0.0;
//��ƫ������ȡƫ��
void GetEPerCount()
{
	unsigned char i=0,j=0;
	//���ĵ��������
	unsigned char TripPointCount = 0;
	// ���������ߵ���С�ڸ���������˵�
	unsigned char FilterNumber = 8;
	// ���䳤���趨
	unsigned char TripLen = 5;//5


	Excursion = 0;
	ValidLineCount=0;
	ValidExcursionCount=0;

	for (i = 0;i < V;i++)
	{
		TripPointPos[i] = 0;	
	}
	//��������ֶ�
	for (i = 0;i < V;i++)
	{
		// ����Ե���ֱ��ȡ��Ե
		if (fabss(RightBlack[i] - LeftBlack[i]) < 5 )     //�������
			{
			  BlackLineData[i] = LeftBlack[i];
		  }
		else{
			//�������ߺ�����ȡ������
			BlackLineData[i] = LeftBlack[i] + (RightBlack[i] - LeftBlack[i])/2;
		}
		// �������ĵ�����ҳ����������䴦,���зֶδ���
		if (i > 0)
		{
			SubValue[i] = BlackLineData[i] - BlackLineData[i-1];
			//�����ж�
			if (fabss(SubValue[i]) > TripLen){
				//��¼����λ�ã��б꣩
				TripPointPos[TripPointCount] = i;
				TripPointCount++;	
			}
		}	
	}//for���� 
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

unsigned char ValidLine[V]={0};//1 - ���ҵ��� 2--���ҵ���  ���߶��ҵ���-3 ���Ҳ�����-0
int NoValidMax=0;//���������Ҳ����߼���

//int RoadWidth[12] = {71,70,69,68,66,65,64,63,62,61,60,59};

//��ȡͼ��������
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
		// ��ʼ������ָ��
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


		// ��ʼ�����
		bFoundLeft = bFoundRight = 0;

		// �������߱�Ե��ȡ
		for (j = 0;j < Hx;j++)
		{
			// ��������
			if (bFoundLeft == 0 && pLeft > 0 && !LeftEnd){// δ�ҵ����Ե��Ѱ��
				//if (pLeft < 1){break;}
				if (ThresholdData[i][pLeft] == 255 && ThresholdData[i][pLeft-1] == 0){
					// �ҵ����Ե
					LeftBlack[i] = pLeft-1;
					if (LeftBlack[i] > 0)
					{
						bFoundLeft = 1;
					}
					
					// ����ұ�ԵҲ�ҵ����˳�
					if (bFoundRight){
						break;
					}
				}
				else{
					pLeft--;
				}
			}// if����--������������Ե
			
			//��������
			if (bFoundRight == 0 && pRight < Hx-1 && !RightEnd){//δ�ҵ��ұ�Ե��Ѱ��
				//if (pRight > ColumnMax - 3){break;}
				if (ThresholdData[i][pRight] == 255 &&  ThresholdData[i][pRight+1] == 0){
					// �ҵ��ұ�Ե
					RightBlack[i] = pRight + 1;
					if (RightBlack[i] < Hx-1)
					{
						bFoundRight = 1;
					}
					
					// ������ԵҲ�ҵ����˳�
					if (bFoundLeft){
						break;
					}
				}
				else{
					pRight++;
				}
			}// if����
		}//for����


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

		//������������ֹ
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
	}//for����-- ��ɨ����


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

//�����߲�������
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

//��ȡ�����߷���
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

//��ȡ����������ƫ�� ��StableNumbersҪ����2
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
	//��������������߷����
	MidLineVariance = 300;
	//���������������ƫ���
	if(MidLineExcursion > 0)
	{
		MidLineExcursion = 40;
	}
	else if(MidLineExcursion < 0)
	{
		MidLineExcursion = -40;	       
	}
}

int TopE1=0;//�������ϰ벿��ƫ��
int TopE2=0;//�������°벿��ƫ��
int TopLen1=0;//�������ϰ벿��ƫ��
int TopLen2=0;//�������ϰ벿��ƫ�� 
int SubBasePoint = 0;//�������°벿�־��׼������ƫ��

void GetSectionParam()   //�������߷ֶ���ȡƫ��
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
signed char TemMidLineData[V];//��ȡ����ֵ����2

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
	//??DD����?2��������DD????
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
        
	//?D??��?��?��������??
	if (BlackEndMax == BlackEndL)////////////////////////////BlackEndMax == BlackEndL
	{
		g_Derict = 1;//����??
	}
	else if (BlackEndMax == BlackEndR)
	{
		g_Derict = 2;//����??
	}
	else if (BlackEndMax == BlackEndM)
	{
		if (fabss(BlackEndL-BlackEndR) < 5)//////////////////////////////////5
		{
			g_Derict = 0;//?y��?��?
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

	//???�¨�?��?��?��???
	for (i = V-1; i > 0;i--)
	{
		if (!EndFlag)
		{
			CrossingStable++;
		}
		//?��??15DD�䨮?D??������?��????��
		if (i > V - 15 )
		{
			// 3?��??����������????
			pLeft = Hx/2;
			pRight = Hx/2;
		}
		else 
		{
			// 3?��??����������????
			pLeft = g_BasePos;
			pRight = g_BasePos;
		}
// 		str.Format("%d pLeft:%d  pRight:%d\r\n",i,pLeft,pRight);
// 		fprintf(pfile,str);
		// 3?��??��??��?����??
		bFoundLeft = bFoundRight = 0;
		for (j = 0;j < Hx;j++)
		{
			// ��������???��
			if (bFoundLeft == 0 && pLeft > 0){// ?��?����?������??��?��?��?��
				//if (pLeft < 1){break;}
				if ((ThresholdData[i][pLeft] == 255 && ThresholdData[i][pLeft-1] == 0) || pLeft == 1){
//					ThresholdData[i][Left0[i]]=255;
					// ?����?������??��
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
							//������?��?D??a??������y
							NoValidLMax=LCount;
						}
					}
					// ��?1?������??�̨�2?����??����?3?
					if (bFoundRight){
						break;
					}
				}
				else{
					pLeft--;
				}
			}// if?����?--�䨮������?����???�¡�??��
			
			//��������???��
			if (bFoundRight == 0 && pRight < Hx-1 ){//?��?����?������??��?��?��?��
				//if (pRight > H - 3){break;}
				if ((ThresholdData[i][pRight] == 255 && ThresholdData[i][pRight+1] == 0) || pRight == Hx-2){
//					ThresholdData[i][RightBlack[i]]=255;
					// ?����?������??��
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
					
					// ��?1?������??�̨�2?����??����?3?
					if (bFoundLeft){
						break;
					}
				}
				else{
					pRight++;
				}
			}// if?����?
		}//for?����?

		//������??��2?��?��??��??0
		if (!bFoundLeft)
		{
			LeftBlack[i]=0;
			//ThresholdData[i][LeftBlack[i]]=100;
		}
		//������??��2?��?��??��??��?�䨮?��
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
			//����??��????��?��??��??DD????e?����������
			if (g_Derict == 1)
			{
				//??��?��??DD?��???????������??��??��??-?����?????��?2��?����DD���?����?aD?��????��?e��?
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
				//??��?��??DD?��???????������??��??��??-?����?????��??��??������??��?�¨�?����������??���䨮D????��?e��?
				else if (LeftBlack[i] + (RightBlack[i]-LeftBlack[i] )/2 > g_BasePos || RightBlack[i] > RightBlack[i+1]+2)
				{
					temLeft = 0;
					temRight = 0;
					bFoundFlag=0;
					//����?��D?��????��?e��?
// 					str.Format("???��?e��?:g_BasePos %d  \r\n",g_BasePos);
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
							//D?��????��?e��?��??-��?��?������?��?��??TD��?��???�¨�?
							EndFlag = 1;
						}
					}
// 					str.Format("?����?D?��????��?e��?:g_BasePos %d  \r\n",g_BasePos);
// 					fprintf(pfile,str);
				}
			}
			
			//��?��?����??��??��??
			if (g_Derict == 2 )
			{
				//??��?��??DD?��???????������??��??��??-?����?????��?2��?����DD���?����?aD?��????��?e��?
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
				//??��?��??DD?��???????������??��??��??-?����?????��??��??������??��?�¨�?����������??���䨮D????��?e��?
				else if (LeftBlack[i] + (RightBlack[i]-LeftBlack[i] )/2 < g_BasePos || LeftBlack[i] < LeftBlack[i+1]-2)
				{
					temLeft = 0;
					temRight = 0;
					bFoundFlag = 0;
					//����?��D?��????��?e��?
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
// 					str.Format("?����?D?��????��?e��?:g_BasePos %d  \r\n",g_BasePos);
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

//?��??��?��????ao?
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
// 		str.Format("213��%d:%d  \r\n",n++,begin);
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
// 		str.Format("213��%d:%d  \r\n",n++,MidPos);
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
		//��?��??DD????��?��DD	
		StableNumbers2++;
		//?DD??????��???1
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
				//?DD?????��?o����?��?
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

//?y��?��?��??��??��|����
void SCProcessing()
{
	int i = 0;
	unsigned char startPos=0,endPos=0;
        int temCount=0,countMax=0,temPos=0;
	unsigned char ProcessFlag=0;
	
	//��?��???��y
	unsigned char TripPointCount = 0;
	unsigned char TripLen = 2;/////////////////////////////////////////////////////////////////////////2
	//��?��?��?��?��??��??
	GetCrossingMidLine();

	for (i = 0;i < V;i++)
	{
		TripPointPos[i] = 0;	
	}
	TripPointCount=0;
	//?��?Y��?��?��???
	for (i = 0;i < V;i++)
	{
		// ?������?DD?��?��?2?��??��3??DD???��?��?��|,??DD��???��|����
		if (i > 0){
			SubValue[i] = LeftBlack[i] - LeftBlack[i-1];
			//��?��??D??
			if (fabss(SubValue[i]) > TripLen || LeftBlack[i] < 2){
				//????��?��?????�ꡧDD������?
				TripPointPos[TripPointCount] = i;
				TripPointCount++;	
			}
		}	
	}//for?����? 
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
	//?��?Y��?��?��???
	for (i = 0;i < V;i++)
	{
		// ?������?DD?��?��?2?��??��3??DD???��?��?��|,??DD��???��|����
		if (i > 0){
			SubValue[i] = RightBlack[i] - RightBlack[i-1];
			//��?��??D??
			if (fabss(SubValue[i]) > TripLen || RightBlack[i] > Hx-3){
				//????��?��?????�ꡧDD������?
				TripPointPos[TripPointCount] = i;
				TripPointCount++;	
			}
		}	
	}//for?����? 
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
		//?ao?������??��
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
	for (i = iStart;i < iEnd;i++)//////////////////////////////////////////////////////////��///////////Hֹ
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
// 			str.Format("��?��?��|����2\r\n");
// 			fprintf(pfile,str);
			if (ValidLine[V-3] == 0)
			{
//				SmoothLeftRight();
				//�̡���??y��?��?��|����
				SCProcessing();
				return;
			}
			else
			{
				//�̡���??y��?��?��|����
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

	//?y��?��?
	if(iCount>4&&g_Derict==0 )/////////////////////////////7
	{	
		SCProcessing();
		return;
	}

	//����???��������??�̨�?��?
	//if (iCount > 4 && g_Derict == 2)
          if ( g_Derict == 2)
	{
// 		str.Format("����??��?��???������??�̡�|����\r\n");
// 		fprintf(pfile,str);

		i = iStart;
                Count1=0;
		Count2=0;
		while (i < iEnd && LeftBlack[i] - LeftBlack[i+1] == 0){i++;}
// 		str.Format("��?��?:%d %d\r\n",i,LeftBlack[i]-LeftBlack[i+1]);
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
				LCrossingTripPos = TripPos;//������??�̨�?��?��??����?
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
				LCrossingTripPos = TripPos;//������??�̨�?��?��??����?
			}
		}
	}

	////����???��������??�̨�?��?��?
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
				RCrossingTripPos = TripPos;//������??�̨�?��?��??����?
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
				RCrossingTripPos = TripPos;//������??�̨�?��?��??����?
			}
		}
	}
	if (bFoundTripPoint)
	{
		GetCrossingMidLine();//��?��?��?��?��??��??
		if (g_Derict == 1)//����??
		{
			pos = TripPos-8;
			while(pos > 0 && (ValidLineR[pos] == 0 || RightBlack[pos] > RightBlack[TripPos])){pos--;} 			
			if (RightBlack[pos-2] < RightBlack[TripPos]/*&&TripPos-(pos-2)<30*/)//?ao??e��?o?������??����???DD?ao?
			{
				//������??��?ao?
				CommonRectificate(&RightBlack[0],pos-2,TripPos);
				ProcessFlag = 1;
			}
			else
			{
				//??DD������???��?��??ao?
				for (i = 0;i < Hx;i++)
				{
					if (ThresholdData[pos-2][i] == 255 && ThresholdData[pos-2][i+1] == 0)
					{
						RightBlack[pos-2]=i;
						break;
					}
				}
				if (RightBlack[pos-2] < RightBlack[TripPos]/*&&TripPos-(pos-2)<30*/)//?ao??e��?o?������??����???DD?ao?
				{
					//������??��?ao?
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
		else if (g_Derict == 2)//����??
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
				//������a��???��?��???DD?ao?
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
	else//?��2?��?��?��?��?
	{
// 		str.Format("?��?����?��?��?��?��|����\r\n");
// 		fprintf(pfile,str);
		GetCrossingMidLine();//��?��?��?��?��??��??
		if (g_Derict == 1)//����??
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
		if(RoadTypeData2[tem] == 0)////////////////////////////////?���̨�
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

		if(RoadTypeData2[tem] == 2)/////////////////////////�䨮S
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

		if(RoadTypeData2[tem] == 2 || RoadTypeData2[tem] == 3 )/////////////////////?����?
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

void HistoryRTProccess()//R��?Road
{
	//int TemCount = 0;
	//int pTem = 0;
	//int i = 0;

	if(StandardRoadType && (RoadType == 0 || RoadType == 1))
	{
		//��?�����̨�����D����?��??-?��?����D?D//������?��??���̨�o��D?S
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

	//�������������̨�����D����3??
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
	//������?��?�����̨�����D�� 3?��2
	if(fabss(Excursion) > 30)
	{
		if(EPerCount >= 1.2 && BlackEndMax < 15)
		{
			temRoadType = 3;	/////////////////////?����?
		}
		else if(EPerCount >= 1.0 && StableNumbers <= 28 && BlackEndMax < 22)
		{
			temRoadType = 2;	///////////////////////�䨮S		   	
		}

	}
	//������?��?�����̨�����D�� 1?��0
      // if(BlackEndMax >= V-1)/////////////////////////////////////////////////////////////���ˡ�
	{
		if(fabss(Excursion)< 5 && MidLineVariance < 2&& EPerCount < 0.1)
		{
			temRoadType = 0;	///////////////////////?���̨�
		}
		else if(fabss(Excursion)< 15 && MidLineVariance < 20)
		{
			temRoadType = 1;//////////////////////////////D?S
		}
	}
	//��?������?��?�����̨���|����
	if(temRoadType==-1)
	{
		StandardRoadType = 0;
		if(BlackEndMax > 33)
		{
			temRoadType = 1;//////////////////////////////D?S
			if (fabss(TopE1-TopE2) > 27 || Mid_K1 > 1.2 || Mid_K2 > 1.2)
			{		   
				temRoadType = 2;///////////////////////�䨮S	
				StandardRoadType = 2;
			}
			if(LastRoadType == 103)
			{
				temRoadType = 103;//////////////////////////?���̨���a��?�̨�
				StraightToBendCount++;
				if(StraightToBendCount > 2)
				{
					temRoadType = 2;///////////////////////�䨮S
					StraightToBendCount=0;	
				}
			}
		}
		else if (BlackEndMax > 20)
		{
			temRoadType = 2;///////////////////////�䨮S
		}
		else 
		{
			temRoadType = 3;/////////////////////?����?
		}
	}
	else             //������?��?�����̨���|����
	{
		StandardRoadType = 1;
	}

	RoadType = temRoadType;//�����̨�����D��?D??3��1|
       HistoryRTProccess();//??�������������̨�����D��D??��??DD��|����
	if(IsStraightToBend() )//?D??��?��?��??����?��?
	{
		RoadType = 103;//////////////////////////?���̨���a��?�̨�
	}
	if(RoadType != 103)
	{
		StraightToBendCount = 0;
	}

	HistoryRoadType[g_Rear] = RoadType;/////////////////////////rearo��??
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
	
	//int MidPos = BlackLineData[V-3] - LeftBlack[V-3]*1.0/2 + (Hx-RightBlack[V-3])*1.0/2;//��?��?��?//////////////��
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
        GetBlackEndParam();//��ȡ���߽�ֹ��
	GetExcursionLine();//��ȡ��ƫ������,����ʶ���ʮ�ֽ����
	GetEPerCount();//����Чƫ����,��ƽ��ÿ��ƫ����	
        StoreFlag = 0;
	GetLMR();//��ȡ���Ե���ұ�Ե��������
        
	//��������µ������ߴ���
	if(StableNumbers > 12)
	{       number=StableNumbers;
		LAverageFilter();//���Ե�˲�
		RAverageFilter();//�ұ�Ե�˲�
		GetFinalMidLine();//ͨ�����ұ�Եȡ��������
		AverageFilter();//�������˲�����
		MidLineCompensate();//�����߲�����
		
		StoreFlag=1;
		StoreMidLine();

		GetSectionParam();//�������߷ֶ���ȡƫ��
		GetMidLineVariance();//��ȡ�����߷���
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



