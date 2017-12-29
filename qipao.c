
#define V  51
#define Hx 70
#define H 320
extern unsigned char Pix_Data[V][H];
extern int b;/////////////////////////////阈值
extern int LeftBlack[V];
extern int RightBlack[V];
extern unsigned char Pix_Data2[V][Hx];
extern unsigned char Pix_Data1[V][Hx];
extern int BlackLineData[V];
int hang=0;
int count=0;///////////////////////////////////////////
int k1=0,k2=0,k3=0,k4=0;//////////////////////////////////
int blackcount=0;//////////////////////////////////
volatile unsigned char IsStartLine =0 ;///////////////////////////////
unsigned char StartLine=0;
extern unsigned char BlackEndMax;




void CheckStartLine()
{
   int start=46,end=50,i=0,j1=0,j2=0,j3=0,j4=0,j=0;
  int value=20;
    k1=0;k2=0;k3=1;k4=0;
    hang=0;
  /*for(i=start;i<end;i++)////////////////////////////////////////////////////////二值化,拉低白点多
  {
    for(j=0;j<69;j++)
    {
      if(Pix_Data2[i][j]>60)/////////////////////////////////////////////////////////////////////////////////////////
        Pix_Data2[i][j]=255;
      else
        Pix_Data2[i][j]=0;
     }    
   }*/

  for(i=start;i<end;i++)
	{
		for(j=0;j<69;j++)
		{
			//白黑白
			if(Pix_Data2[i][j]-Pix_Data2[i][j+1] > value && Pix_Data2[i][j+2]-Pix_Data2[i][j+1] > value )	
			{
				Pix_Data2[i][j+1]=255;
			}
			//黑白黑
			else if(Pix_Data2[i][j+1]-Pix_Data2[i][j] > value && Pix_Data2[i][j+1]-Pix_Data2[i][j+2] > value )
			{
				Pix_Data2[i][j+1]=255;	//////////////////////////////////////////////因为黑点太多所以保持
			}
                        
                     if(Pix_Data2[i][j]==255&&Pix_Data2[i][j+1]==0&&Pix_Data2[i][j+2]==0&&Pix_Data2[i][j+3]==255 ) 
                     {Pix_Data2[i][j+1]=255;Pix_Data2[i][j+2]=255;}
                                         // if(Pix_Data2[i][j]==255&&Pix_Data2[i][j+1]==0&&Pix_Data2[i][j+2]==0&&Pix_Data2[i][j+3]==0&&Pix_Data2[i][j+4]==255 ) 
                    // {Pix_Data2[i][j+1]=255;Pix_Data2[i][j+2]=255;Pix_Data2[i][j+3]=255;}
		}
	}

  for(i=start;i<end;i++)
 {
    k1=0;k2=0;k3=1;k4=0;
    count=0;
    if(((LeftBlack[i]>=10&&LeftBlack[i]<=25)&&(RightBlack[i]>=45&&RightBlack[i]<=56))||((LeftBlack[i]>=17&&LeftBlack[i]<=27)&&(RightBlack[i]>=48&&RightBlack[i]<=57)))//((LeftBlack[i]>=21&&LeftBlack[i]<=29)&&(RightBlack[i]>=46&&RightBlack[i]<=60))||||(((LeftBlack[i]>=10&&LeftBlack[i]<=18)&&(RightBlack[i]>=44&&RightBlack[i]<=49)))
  {     
    for(j1=LeftBlack[i];j1<BlackLineData[i];j1++)/////////////////////左 
    {
      if(Pix_Data2[i][j1]-Pix_Data2[i][j1+1] > value && Pix_Data2[i][j1]-Pix_Data2[i][j1+2] > value && Pix_Data2[i][j1]-Pix_Data2[i][j1+3] > value&& Pix_Data2[i][j1]-Pix_Data2[i][j1+4]> value)//
      {//////
        k1=j1;
        count=1;      
      for(j2=BlackLineData[i]-1;j2>LeftBlack[i];j2--)////////////&&Pix_Data2[i][j2-2]==0&&Pix_Data2[i][j2-5]==0&&Pix_Data2[i][j2-4]==0&&Pix_Data2[i][j2-3]==0
      {
        if(Pix_Data2[i][j2]==0&&Pix_Data2[i][j2-1]==0)
        {
          k2=j2;
          count=2;
          break;          
        }
       ////
      
     }
     break;
    }
   }

    for(j3=RightBlack[i];j3>BlackLineData[i];j3--) ////////////////////&& Pix_Data2[i][j3]-Pix_Data2[i][j3-5] > value
    {
      if(Pix_Data2[i][j3]-Pix_Data2[i][j3-1] > value && Pix_Data2[i][j3]-Pix_Data2[i][j3-2] > value&& Pix_Data2[i][j3]-Pix_Data2[i][j3-3] > value&& Pix_Data2[i][j3]-Pix_Data2[i][j3-4] > value )//
      {
        k3=j3;
        count=3;
      for(j4=BlackLineData[i]+1;j4<RightBlack[i];j4++)//&&Pix_Data2[i][j4+2]==0///////////&&Pix_Data2[i][j4+4]==0&&Pix_Data2[i][j4+5]==0&&Pix_Data2[i][j4+3]==0
      { 
        if(Pix_Data2[i][j4]==0&&Pix_Data2[i][j4+1]==0)
        {
          k4=j4;
          count=4;
          break;
        }            
      ///////
     
     }
      break;
    }
   }
    if(count==4)
    {
     /* if(hang==1)////////////////////////////如果想连续两行满足要求就加此(最好要此)
      {
        hang=2;
        break;
      }
      else*/
        hang=1;
        break;
    }
    else
      hang=0;
   }
  }
  if(hang==1)  
    IsStartLine=1;
    // hang=0;
   //count=0;
  
}





